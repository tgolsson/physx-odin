
mod comment;
mod enums;
mod functions;
mod record;

use crate::{writesln, writes, consumer::{AstConsumer, Builtin, EnumBinding, FuncBinding, RecBinding}};
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
        cpp: &mut impl Write,

    ) -> anyhow::Result<()> {
		let mut cc = File::create(rr.join("foo/physx_generated.odin"))?;
		let mut ch = File::create(rr.join("foo/physx_generated.odin"))?;
		let mut che = File::create(rr.join("foo/physx_generated.odin"))?;
		writesln!(ch, "package physx");

        self.generate_cpp(ast, cpp)?;
        self.generate_c(ast, &mut cc, &mut ch, &mut che)?;

        Ok(())
    }


    pub fn generate_cpp(&self, ast: &AstConsumer<'_>, out: &mut impl Write) -> anyhow::Result<()> {
        self.generate_size_asserts(ast, out)?;
        self.generate_cpp_functions(ast, out, 0)?;

        Ok(())
    }

    /// Generates the static assert code used to verify that every structgen
    /// POD type is the same size as the C++ type it is wrapping
    pub fn generate_size_asserts(
        &self,
        ast: &AstConsumer<'_>,
        out: &mut impl Write,
    ) -> anyhow::Result<()> {
        writeln!(
            out,
            "using namespace physx;\n#include \"structgen_out.hpp\"\n"
        )?;

        for rec in ast.recs.iter().filter_map(|rb| {
            if let RecBinding::Def(def) = rb {
                if (self.record_filter)(rb) {
                    return Some(def);
                }
            }

            None
        }) {
            let name = rec.name;
            writeln!(out, "static_assert(sizeof(physx::{name}) == sizeof(physx_{name}_Pod), \"POD wrapper for `physx::{name}` has incorrect size\");")?;
        }

        writeln!(out)?;

        Ok(())
    }

    /// Generates the C functions used to convert between the C bridge types
    /// and calls into the C++ code
    pub fn generate_cpp_functions(
        &self,
        ast: &AstConsumer<'_>,
        out: &mut impl Write,
        level: u32,
    ) -> anyhow::Result<()> {
        let indent = Indent(level);

        writeln!(out, "{indent}extern \"C\" {{")?;
        let mut acc = String::new();
        for func in ast.funcs.iter().filter(|fb| (self.func_filter)(fb)) {
            acc.clear();
            func.emit_cpp(&mut acc, level + 1)?;
            writeln!(out, "{acc}")?;
        }
        writeln!(out, "{indent}}}")?;

        Ok(())
    }

    pub fn generate_c(&self, ast: &AstConsumer<'_>, cc: &mut impl Write, ch: &mut impl Write, che: &mut impl Write) -> anyhow::Result<()> {
        let level = 0;

		writesln!(ch, "#include <stdint.h>");
		writesln!(ch, "#include <stddef.h>");
        self.generate_c_enums(ast, cc, che, level)?;
        self.generate_c_records(ast, cc, ch)?;
        self.generate_c_functions(ast,cc,ch, level)?;

        Ok(())
    }

    pub fn generate_c_enums(
        &self,
        ast: &AstConsumer<'_>,
        _cc: &mut impl Write,
        ch: &mut impl Write,
        level: u32,
    ) -> anyhow::Result<u32> {
        let mut fiter = ast.flags.iter().peekable();
        let mut acc = String::new();

		writesln!(ch, "#pragma once");
		writesln!(ch, "#include <stdint.h>");
		writesln!(ch, "#include <stddef.h>");

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

            if (self.enum_filter)(eb) {
                Some((eb, fb))
            } else {
                None
            }
        }) {
            if !acc.is_empty() {
                acc.clear();
                writesln!(acc);
            }

            enum_binding.emit_c(&mut acc, level);

            if let Some((builtin, default)) = INT_ENUMS.iter().find_map(|(name, bi, def)| {
                if *name == enum_binding.name {
                    Some((*bi, *def))
                } else {
                    None
                }
            }) {
                writesln!(acc);
                enum_binding.emit_c_conversion(&mut acc, level, builtin, default);
            }

            if let Some(flags) = flags_binding {
                writesln!(acc);
                flags.emit_c(enum_binding, &mut acc, level);
            }

            write!(ch, "{acc}")?;
        }

        Ok((ast.enums.len() + ast.flags.len()) as u32)
    }

    pub fn generate_c_records(
        &self,
        ast: &AstConsumer<'_>,
        cc: &mut impl Write,
        ch: &mut impl Write,
    ) -> anyhow::Result<u32> {
        let mut num = 0;
        let mut acc = String::new();

		writesln!(ch, "#include \"physx_generated_enums.h\"");

        for rec in ast.recs.iter().filter(|rb| (self.record_filter)(rb)) {
            acc.clear();
            writesln!(acc);

            match rec {
                RecBinding::Def(def) => {
                    if def.emit_c(&mut acc, 0) {
                        num += 1;
                        write!(ch, "{acc}")?;
                    }
                }
                RecBinding::Forward(forward) => {

                    forward.emit_c(&mut acc, 0);
                    write!(ch, "{acc}")?;
                    num += 1;

                }
            }
        }

        Ok(num)
    }

    pub fn generate_c_functions(
        &self,
        ast: &AstConsumer<'_>,
        _cc: &mut impl Write,
        ch: &mut impl Write,
        level: u32,
    ) -> anyhow::Result<u32> {

        let mut acc = String::new();
        for func in ast.funcs.iter().filter(|fb| (self.func_filter)(fb)) {
            acc.clear();
            func.emit_c(&mut acc, level + 1);
            writeln!(ch, "{acc}")?;
        }


        Ok(0)
    }
}

struct CIdent<'ast>(&'ast str);

impl<'ast> fmt::Display for CIdent<'ast> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        static KEYWORDS: &[&str] = &["box", "type", "ref"];

        f.write_str(self.0)?;

        if KEYWORDS.contains(&self.0) {
            f.write_str("_")?;
        }

        Ok(())
    }
}
