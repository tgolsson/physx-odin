mod comment;
mod enums;
mod functions;
mod record;

use crate::{
    consumer::{AstConsumer, Builtin, EnumBinding, FuncBinding, OdinType, QualType, RecBinding},
    writesln, StructMetadataList,
};
use std::{
    fmt,
    fs::{create_dir_all, exists, File},
    io::Write,
};

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
        let mut odin = File::create(rr.join("physx_generated.odin"))?;
        writesln!(odin, "package physx");

        let test_dir = rr.join("tests");
        if !exists(&test_dir)? {
            create_dir_all(&test_dir)?;
        }
        let mut tests = File::create(test_dir.join("physx_generated.odin"))?;
        writesln!(tests, "package tests");
        writesln!(tests, "import  \"core:testing\"");
        writesln!(tests, "import physx \"..\"");
        self.generate_odin(ast, &sizes, &mut odin, &mut tests)?;

        Ok(())
    }

    pub fn generate_odin(
        &self,
        ast: &AstConsumer<'_>,
        metadata: &StructMetadataList,
        odin: &mut impl Write,
        tests: &mut impl Write,
    ) -> anyhow::Result<()> {
        let level = 0;

        writesln!(odin, "import _c \"core:c\"");

        self.generate_odin_enums(ast, odin, level)?;
        self.generate_odin_records(ast, metadata, odin)?;
        self.generate_odin_functions(ast, odin, level)?;
        self.generate_tests(ast, metadata, tests)?;
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
            "when ODIN_OS == .Linux do foreign import libphysx \"libphysx.so\"\n"
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

    pub fn generate_tests(
        &self,
        ast: &AstConsumer<'_>,
        metadata: &StructMetadataList,
        tests: &mut impl Write,
    ) -> anyhow::Result<u32> {
        let mut num = 0;
        let mut acc = String::new();

        for rec in ast.recs.iter() {
            acc.clear();
            writesln!(acc);

            match rec {
                RecBinding::Def(def) => {
                    def.emit_odin_test(&mut acc, metadata, 0);
                    num += 1;
                    write!(tests, "{acc}")?;
                }
                _ => {}
            }
        }

        Ok(num)
    }
}

struct OdinIdent<'ast>(&'ast str);

impl<'ast> fmt::Display for OdinIdent<'ast> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        static KEYWORDS: &[&str] = &["matrix", "context"];

		let stripped = self.0.strip_prefix("Px").unwrap_or(self.0);

		if stripped.starts_with("1D") {
			f.write_str(&stripped.replace("1D", "OneD"))?;
		} else {
			f.write_str(stripped)?;
		}

        if KEYWORDS.contains(&self.0) {
            f.write_str("_")?;
		}

        Ok(())
    }
}


impl<'qt, 'ast> fmt::Display for OdinType<'qt, 'ast> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {

            QualType::Pointer { pointee, .. } => {
               match pointee.odin_type().0 {
                   QualType::Builtin(Builtin::Void) => write!(f, "rawptr")?,
                    _ => write!(f, "^{}", pointee.odin_type())?,
                }

                Ok(())
            }
            QualType::Reference { pointee, .. } => {
                match pointee.odin_type().0 {
                    QualType::Builtin(Builtin::Void) => write!(f, "rawptr")?,
                    _ => write!(f, "^{}", pointee.odin_type())?,
                }

                Ok(())
            }
            QualType::Builtin(bi) => f.write_str(bi.odin_type()),
            QualType::FunctionPointer => f.write_str("rawptr"),
            QualType::Array { element, len } => {
                panic!("C array `{}[{len}]` breaks the pattern of every other type by have elements on both sides of an identifier", element.odin_type());
            }
            QualType::Enum { name, .. } => {
				write!(f, "{}", OdinIdent(name))
			},
			QualType::Flags { name, .. } => {
				write!(f, "{}_Set", OdinIdent(name))
            }
            QualType::Record { name } => write!(f, "{}", OdinIdent(name)),
            QualType::TemplateTypedef { name } => write!(f, "{}", OdinIdent(name)),
        }
    }
}
