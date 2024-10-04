use super::{Indent, SG, UOF};
use crate::consumer::{FieldBinding, QualType};
use crate::{writesln, writes};

impl<'ast> crate::consumer::RecBindingDef<'ast> {
    pub fn emit_c(&self, w: &mut String, level: u32) -> bool {
        if self.calc_layout {
            return false;
        }

        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));

		writesln!(
			w,
            "{indent}typedef {} physx_{} physx_{};",
            if is_union { "union" } else { "struct" },
            self.name,             self.name
        );

        writesln!(
            w,
            "{indent}{} physx_{} {{",
            if is_union { "union" } else { "struct" },
            self.name
        );

        if self.has_vtable {
            writesln!(w, "{indent1}void* _vtable;");
        }

        for field in &self.fields {
			if let QualType::Array { element, len } = & field.kind {
				writesln!(w, "{indent1}{} {}{};", element.real_ctype(), field.name, len);
			} else {
				writesln!(w, "{indent1}{} {};", field.kind.real_ctype(), field.name);
			}
        }

        writesln!(w, "{indent}}};");
        true
    }
}

impl<'ast> crate::consumer::RecBindingForward<'ast> {
    pub fn emit_c(&self, w: &mut String, level: u32) {
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        writesln!(w, "{indent}typedef struct physx_{} {{", self.name);
        writesln!(w, "{indent1}char _unused [0];");
        writesln!(w, "}} physx_{};", self.name);
    }
}
