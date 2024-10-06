use super::Indent;
use crate::consumer::{FieldBinding, QualType};
use crate::{writes, writesln, StructMetadata};

/// The variable name of `PodStructGen` in the structgen program
const SG: &str = "sg";
/// The name of the macro used to calculate a field's offset in the structgen program
const UOF: &str = "unsafe_offsetof";

impl<'ast> crate::consumer::RecBindingDef<'ast> {
    pub fn emit_odin(&self, w: &mut String, meta: Option<&StructMetadata>, level: u32) -> bool {
        if self.calc_layout {
            self.emit_odin_calc_layout(w, meta, level)
        } else {
            self.emit_odin_raw(w, level)
        }
    }

    pub fn emit_odin_calc_layout(
        &self,
        w: &mut String,
        meta: Option<&StructMetadata>,
        level: u32,
    ) -> bool {
        let meta = meta.unwrap();
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));

        writesln!(
            w,
            "{indent}{} :: struct {}{{",
            self.name,
            if is_union { "#raw_union " } else { "" },
        );

        let mut count_fields = if self.has_vtable {
            writesln!(w, "{indent1}_vtable: rawptr,");
            1
        } else {
            0
        };

		let mut pad_index = 0;
        for emitted_field in &meta.fields {
			if emitted_field.name == "PAD"{

                writesln!(w, "{indent1}_pad{}: [{}]u8,", pad_index, emitted_field.size);
				pad_index += 1;
				continue;
			}
			let name = if emitted_field.name.ends_with(']') {
				emitted_field.name[..emitted_field.name.find('[').unwrap()].to_owned()
			} else {
				emitted_field.name.clone()
			};
			dbg!(&name);
			let field = self.fields.iter().find(|f| f.name == name).unwrap();
            if !field.is_public || field.is_reference {
                continue;
            }

            count_fields += 1;
            if let QualType::Array { element, len } = &field.kind {
                if let QualType::Array {
                    element: inner,
                    len: len1,
                } = &**element
                {
                    writesln!(
                        w,
                        "{indent1}{}: [{}][{}]{},",
                        field.name,
                        len,
                        len1,
                        inner.odin_type(),
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}{}: [{}]{},",
                        field.name,
                        len,
                        element.odin_type(),
                    );
                }
            } else {
                writesln!(w, "{indent1}{}: {},", field.name, field.kind.odin_type());
            }
        }

        if count_fields == 0 {
            writesln!(w, "{indent1}unused0: [1]u8,");
        }

        writesln!(w, "{indent}}}");
        true
    }

    pub fn emit_odin_raw(&self, w: &mut String, level: u32) -> bool {
        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(
            w,
            "{indent}{} :: struct {}{{",
            self.name,
            if is_union { "#raw_union " } else { "" },
        );

        if self.ast.definition_data.is_none() {
            return true;
        }

        let mut count_fields = if self.has_vtable {
            writes!(w, "{indent1}vtable_: rawptr,\n");
            1
        } else {
            0
        };

        for field in &self.fields {
            if !field.is_public || field.is_reference {
                continue;
            }

            count_fields += 1;
            if let QualType::Array { element, len } = &field.kind {
                if let QualType::Array {
                    element: inner,
                    len: len1,
                } = &**element
                {
                    writesln!(
                        w,
                        "{indent1}{}: [{}][{}]{},",
                        field.name,
                        len,
                        len1,
                        inner.odin_type(),
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}{}: [{}]{},",
                        field.name,
                        len,
                        element.odin_type(),
                    );
                }
            } else {
                writesln!(w, "{indent1}{}: {},", field.name, field.kind.odin_type());
            }
        }

        if count_fields == 0 {
            writesln!(w, "{indent1}_unused0: [1]u8,");
        }

        writes!(w, "}};\n");
        true
    }
}

impl<'ast> crate::consumer::RecBindingForward<'ast> {
    pub fn emit_odin(&self, w: &mut String, level: u32) {
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        writesln!(w, "{indent}{} :: struct {{", self.name);
        writesln!(w, "{indent1}_unused: [0]u8,");
        writesln!(w, "}}");
    }
}
