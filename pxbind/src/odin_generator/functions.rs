use super::Indent;
use crate::consumer::{functions::*, QualType};
use crate::{writesln, writes};

const RET: &str = "return_val";
const RET_POD: &str = "return_val_pod";

impl<'ast> Param<'ast> {

}

impl<'ast> FuncBinding<'ast> {
    pub(super) fn emit_c(&self, writer: &mut String, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_c(writer, level);
        }

        let indent = Indent(level);

        let mut acc = String::new();

        if let Some(ret) = &self.ret {
            writes!(acc, "{} ", ret.c_type());
        } else {
            writes!(acc, "void ");
        }

        writes!(acc, "{indent} physx_{}(", self.name);

        for (i, param) in self.params.iter().enumerate() {
            let sep = if i > 0 { ", " } else { "" };

            writes!(
                acc,
                "{sep}{} {}",
                param.kind.c_type(),
                super::CIdent(&param.name),
            );
        }

        writesln!(writer, "{acc});");
    }
}
