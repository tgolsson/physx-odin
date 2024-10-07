mod comment;
mod enums;
mod functions;
mod record;

use crate::{
    consumer::{AstConsumer, Builtin, EnumBinding, FuncBinding, RecBinding},
	writesln, StructMetadataList,
};
use std::{fmt, fs::File, io::Write};

/// It's impossible (I believe) with Rust's format strings to have the width
/// of the alignment be dynamic, so we just uhhh...be lame
struct Indent(u32);

impl fmt::Display for Indent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for _ in 0..self.0 {
            f.write_str("    ")?;
        }

        Ok(())
    }
}

pub struct Generator {
    pub record_filter: Box<dyn Fn(&RecBinding<'_>) -> bool>,
    pub enum_filter: Box<dyn Fn(&EnumBinding<'_>) -> bool>,
    pub func_filter: Box<dyn Fn(&FuncBinding<'_>) -> bool>,
}

impl Default for Generator {
    fn default() -> Self {
        Self {
            record_filter: Box::new(|_rb| true),
            enum_filter: Box::new(|_eb| true),
            func_filter: Box::new(|_fb| true),
        }
    }
}

impl Generator {
    pub fn generate_all(
        &self,
        ast: &AstConsumer<'_>,
        rr: std::path::PathBuf,
        sizes: super::StructMetadataList,
    ) -> anyhow::Result<()> {
        let mut odin = File::create(rr.join("src/physx_generated.odin"))?;
        writesln!(odin, "package physx");
        self.generate_odin(ast, &sizes, &mut odin)?;

        Ok(())
    }

    pub fn generate_odin(
        &self,
        ast: &AstConsumer<'_>,
        metadata: &StructMetadataList,
        odin: &mut impl Write,
    ) -> anyhow::Result<()> {
        let level = 0;

        writesln!(odin, "import _c \"core:c\"");
        writesln!(odin, "import  \"core:testing\"");
        self.generate_odin_enums(ast, odin, level)?;
        self.generate_odin_records(ast, metadata, odin)?;
        self.generate_odin_functions(ast, odin, level)?;

        Ok(())
    }

    pub fn generate_odin_enums(
        &self,
        ast: &AstConsumer<'_>,
        odin: &mut impl Write,
        level: u32,
    ) -> anyhow::Result<u32> {
        let mut fiter = ast.flags.iter().peekable();
        let mut acc = String::new();

        const INT_ENUMS: &[(&str, Builtin, &str)] = &[
            ("PxConcreteType", Builtin::UShort, "Undefined"),
            ("PxD6Drive", Builtin::USize, "Count"),
        ];

        for (enum_binding, flags_binding) in ast.enums.iter().enumerate().filter_map(|(i, eb)| {
            let fb = if fiter.peek().map_or(false, |f| f.enums_index == i) {
                fiter.next()
            } else {
                None
            };

            Some((eb, fb))
        }) {
            if !acc.is_empty() {
                acc.clear();
                writesln!(acc);
            }

            let is_flags = flags_binding.is_some();
            enum_binding.emit_odin(&mut acc, is_flags, level);

            if let Some(flags) = flags_binding {
                writesln!(acc);
                flags.emit_odin(enum_binding, &mut acc, level);
            }

            write!(odin, "{acc}")?;
        }

        Ok((ast.enums.len() + ast.flags.len()) as u32)
    }

    pub fn generate_odin_records(
        &self,
        ast: &AstConsumer<'_>,
        metadata: &StructMetadataList,
        odin: &mut impl Write,
    ) -> anyhow::Result<u32> {
        let mut num = 0;
        let mut acc = String::new();

        for rec in ast.recs.iter().filter(|rb| (self.record_filter)(rb)) {
            acc.clear();
            writesln!(acc);

            match rec {
                RecBinding::Def(def) => {
                    if def.emit_odin(&mut acc, ast, metadata, 0) {
                        num += 1;
                        write!(odin, "{acc}")?;
                    }
                }
                RecBinding::Forward(forward) => {
                    if matches!(ast.classes.get(forward.name), Some(None)) {
                        forward.emit_odin(&mut acc, 0);
                        write!(odin, "{acc}")?;
                        num += 1;
                    }
                }
            }
        }

        Ok(num)
    }

    pub fn generate_odin_functions(
        &self,
        ast: &AstConsumer<'_>,
        odin: &mut impl Write,
        level: u32,
    ) -> anyhow::Result<u32> {
        writesln!(
            odin,
            "when ODIN_OS == .Linux do foreign import libphysx \"physx.so\"\n"
        );
        writesln!(odin, "@(default_calling_convention = \"c\")");
        writesln!(odin, "foreign libphysx {{");
        let mut acc = String::new();
        for func in ast.funcs.iter() {
            acc.clear();
            func.emit_odin(&mut acc, ast, level + 1);
            writeln!(odin, "{acc}")?;
        }
        writesln!(odin, "}}");
        Ok(0)
    }
}

struct OdinIdent<'ast>(&'ast str);

impl<'ast> fmt::Display for OdinIdent<'ast> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        static KEYWORDS: &[&str] = &["matrix", "context"];

        f.write_str(self.0)?;

        if KEYWORDS.contains(&self.0) {
            f.write_str("_")?;
        }

        Ok(())
    }
}
