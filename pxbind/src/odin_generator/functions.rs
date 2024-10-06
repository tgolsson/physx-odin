use super::Indent;
use crate::consumer::{functions::*, QualType};
use crate::{writesln, writes};

const RET: &str = "return_val";
const RET_POD: &str = "return_val_pod";

impl<'ast> Param<'ast> {

}

impl<'ast> FuncBinding<'ast> {
    pub(super) fn emit_odin(&self, writer: &mut String, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_odin(writer, level);
        }

        let indent = Indent(level);

        let mut acc = String::new();

        writes!(acc, "{indent}{} :: proc(", self.name);

        for (i, param) in self.params.iter().enumerate() {
            let sep = if i > 0 { ", " } else { "" };

            writes!(
                acc,
                "{sep}{}: {}",
                super::OdinIdent(&param.name),
                param.kind.odin_type(),
            );
        }

		if let Some(ret) = &self.ret {
            writes!(acc, ") -> {}", ret.odin_type());
        } else {
            writes!(acc, ")");
        }

        writesln!(writer, "{acc} ---");
    }
}
