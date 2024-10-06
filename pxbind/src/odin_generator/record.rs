use super::Indent;
use crate::consumer::{FieldBinding, QualType};
use crate::{writes, writesln, FieldMetadata, StructMetadata, StructMetadataList};

/// The variable name of `PodStructGen` in the structgen program
const SG: &str = "sg";
/// The name of the macro used to calculate a field's offset in the structgen program
const UOF: &str = "unsafe_offsetof";

impl<'ast> crate::consumer::RecBindingDef<'ast> {
    pub fn emit_odin(&self, w: &mut String, meta: &StructMetadataList, level: u32) -> bool {
        if self.calc_layout {
            self.emit_odin_calc_layout(w, meta, level)
        } else {
            self.emit_odin_raw(w, level)
        }
    }

    pub fn emit_odin_calc_layout(
        &self,
        w: &mut String,
        metalist: &StructMetadataList,
        level: u32,
    ) -> bool {
        let meta = metalist
            .structs
            .iter()
            .find(|s| s.name == self.name)
            .unwrap();
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));
        let base_fields = if let Some(base) = self.bases.get(0) {
            if let Some(base) = metalist.structs.iter().find(|s| s.name == base.name) {
                &base.fields
            } else {
                &[] as &[FieldMetadata]
            }
        } else {
            &[]
        };

        writesln!(
            w,
            "{indent}{} :: struct {}{{",
            self.name,
            if is_union { "#raw_union " } else { "" },
        );

        for base in &self.bases {
            writesln!(w, "{indent1}using _: {},", base.name);
        }

        let mut count_fields = 0;
        let mut pad_index = 0;

        for (idx, emitted_field) in meta.fields.iter().enumerate() {
            if base_fields.len() > idx {
                let base_field = &base_fields[idx];
                if base_field.name != "PAD" {
                    assert_eq!(
                        base_field.name, emitted_field.name,
                        "{}{:?} {base_field:?} <-> {emitted_field:?}",
                        self.name, self.bases
                    );
                    assert_eq!(
                        base_field.size, emitted_field.size,
                        "{}{:?} {base_field:?} <-> {emitted_field:?}",
                        self.name, self.bases
                    );
                    assert_eq!(
                        base_field.offset, emitted_field.offset,
                        "{}{:?} {base_field:?} <-> {emitted_field:?}",
                        self.name, self.bases
                    );
                } else {
					if emitted_field.name == "PAD" {
						dbg!(emitted_field, base_field);
						if emitted_field.size > base_field.size {
							writesln!(w, "{indent1}_pad{}: [{}]u8,", base_fields.len() + idx, emitted_field.size - base_field.size);
						}
					}
				}

                continue;
            }
            if emitted_field.name == "PAD"  {
				 writesln!(w, "{indent1}_pad{}: [{}]u8,", base_fields.len() + idx, emitted_field.size);

				count_fields += 1;
				pad_index += 1;

                continue;
            }
            let name = if emitted_field.name.ends_with(']') {
                emitted_field.name[..emitted_field.name.find('[').unwrap()].to_owned()
            } else {
                emitted_field.name.clone()
            };

            let field = self
                .fields
                .iter()
                .chain(&self.base_fields)
                .find(|f| f.name == name)
                .unwrap();

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

        if count_fields == 0 && self.bases.is_empty() {
            writesln!(w, "{indent1}unused0: [1]u8,");
        }

        writesln!(w, "{indent}}}");

        writesln!(
            w,
            "#assert(size_of({}) == {}, \"Wrong size for type {}, expected {}\")",
            self.name,
            meta.size,
            self.name,
            meta.size
        );
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

        for base in &self.bases {
            writesln!(w, "{indent1}using _: {},", base.name);
        }

        let mut count_fields = if self.has_vtable && self.bases.is_empty() {
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

        if count_fields == 0 && self.bases.is_empty() {
            writesln!(w, "{indent1}_unused0: [1]u8,");
        }

        writes!(w, "}};\n");
        true
    }
}

impl<'ast> crate::consumer::RecBindingForward<'ast> {
    pub fn emit_odin(&self, w: &mut String, level: u32) {
        let indent = Indent(level);

        writesln!(w, "{indent}{} :: distinct rawptr ", self.name);
    }
}
