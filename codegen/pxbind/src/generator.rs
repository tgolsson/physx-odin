mod comment;
mod enums;
mod functions;
mod record;

use crate::{
    consumer::{EnumBinding, FuncBinding, RecBinding},
    type_db::TypeDB,
};
use std::io::Write;

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
        _ast: &TypeDB,
        _structgen: &mut impl Write,
        _cpp: &mut impl Write,
        _rust: &mut impl Write,
    ) -> anyhow::Result<()> {
        // self.generate_structgen(ast, structgen)?;
        // self.generate_cpp(ast, cpp)?;
        // self.generate_rust(ast, rust)?;

        Ok(())
    }

    // pub fn generate_rust(&self, ast: &AstConsumer<'_>, w: &mut impl Write) -> anyhow::Result<()> {
    //     let level = 0;

    //     self.generate_rust_enums(ast, w, level)?;
    //     self.generate_rust_records(ast, w)?;
    //     self.generate_rust_functions(ast, w, level)?;

    //     Ok(())
    // }

    // pub fn generate_rust_enums(
    //     &self,
    //     ast: &AstConsumer<'_>,
    //     writer: &mut impl Write,
    //     level: u32,
    // ) -> anyhow::Result<u32> {
    //     let mut fiter = ast.flags.iter().peekable();
    //     let mut acc = String::new();

    //     const INT_ENUMS: &[(&str, Builtin, &str)] = &[
    //         ("PxConcreteType", Builtin::UShort, "Undefined"),
    //         ("PxD6Drive", Builtin::USize, "Count"),
    //     ];

    //     for (enum_binding, flags_binding) in ast.enums.iter().enumerate().filter_map(|(i, eb)| {
    //         let fb = if fiter.peek().map_or(false, |f| f.enums_index == i) {
    //             fiter.next()
    //         } else {
    //             None
    //         };

    //         if (self.enum_filter)(eb) {
    //             Some((eb, fb))
    //         } else {
    //             None
    //         }
    //     }) {
    //         if !acc.is_empty() {
    //             acc.clear();
    //             writesln!(acc);
    //         }

    //         enum_binding.emit_rust(&mut acc, level);

    //         if let Some((builtin, default)) = INT_ENUMS.iter().find_map(|(name, bi, def)| {
    //             if *name == enum_binding.name {
    //                 Some((*bi, *def))
    //             } else {
    //                 None
    //             }
    //         }) {
    //             writesln!(acc);
    //             enum_binding.emit_rust_conversion(&mut acc, level, builtin, default);
    //         }

    //         if let Some(flags) = flags_binding {
    //             writesln!(acc);
    //             flags.emit_rust(enum_binding, &mut acc, level);
    //         }

    //         write!(writer, "{acc}")?;
    //     }

    //     Ok((ast.enums.len() + ast.flags.len()) as u32)
    // }

    // pub fn generate_rust_records(
    //     &self,
    //     ast: &AstConsumer<'_>,
    //     writer: &mut impl Write,
    // ) -> anyhow::Result<u32> {
    //     let mut num = 0;
    //     let mut acc = String::new();

    //     for rec in ast.recs.iter().filter(|rb| (self.record_filter)(rb)) {
    //         acc.clear();
    //         writesln!(acc);

    //         match rec {
    //             RecBinding::Def(def) => {
    //                 if def.emit_rust(&mut acc, 0) {
    //                     num += 1;
    //                     write!(writer, "{acc}")?;
    //                 }
    //             }
    //             RecBinding::Forward(forward) => {
    //                 if matches!(ast.classes.get(forward.name), Some(None)) {
    //                     forward.emit_rust(&mut acc, 0);
    //                     write!(writer, "{acc}")?;
    //                     num += 1;
    //                 }
    //             }
    //         }
    //     }

    //     Ok(num)
    // }

    // pub fn generate_rust_functions(
    //     &self,
    //     ast: &AstConsumer<'_>,
    //     w: &mut impl Write,
    //     level: u32,
    // ) -> anyhow::Result<u32> {
    //     let indent = Indent(level);

    //     writeln!(w, "{indent}extern \"C\" {{")?;
    //     let mut acc = String::new();
    //     for func in ast.funcs.iter().filter(|fb| (self.func_filter)(fb)) {
    //         acc.clear();
    //         func.emit_rust(&mut acc, level + 1);
    //         writeln!(w, "{acc}")?;
    //     }
    //     writeln!(w, "{indent}}}")?;

    //     Ok(0)
    // }
}

// struct RustIdent<'ast>(&'ast str);

// impl<'ast> fmt::Display for RustIdent<'ast> {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         static KEYWORDS: &[&str] = &["box", "type", "ref"];

//         f.write_str(self.0)?;

//         if KEYWORDS.contains(&self.0) {
//             f.write_str("_")?;
//         }

//         Ok(())
//     }
// }
