use crate::consumer::functions::*;

impl<'ast> FuncBinding<'ast> {
    // pub(super) fn emit_rust(&self, writer: &mut String, level: u32) {
    //     if let Some(com) = &self.comment {
    //         com.emit_rust(writer, level);
    //     }

    //     let indent = Indent(level);

    //     let mut acc = String::new();
    //     writes!(acc, "{indent}pub fn {}(", self.name);

    //     for (i, param) in self.params.iter().enumerate() {
    //         // While Rust allows trailing commas in function signatures, it's
    //         // kind of ugly
    //         let sep = if i > 0 { ", " } else { "" };

    //         writes!(
    //             acc,
    //             "{sep}{}: {}",
    //             super::RustIdent(&param.name),
    //             param.kind.rust_type()
    //         );
    //     }

    //     if let Some(ret) = &self.ret {
    //         writes!(acc, ") -> {};", ret.rust_type());
    //     } else {
    //         writes!(acc, ");");
    //     }

    //     writesln!(writer, "{acc}");
    // }
}
