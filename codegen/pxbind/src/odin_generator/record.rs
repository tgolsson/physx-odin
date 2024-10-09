use super::Indent;
use crate::odin_generator::OdinIdent;
use crate::type_db::TypeDB;
use crate::type_db::{QualTypeValue, RecBindingDef, RecBindingValue};
use crate::{writes, writesln, FieldMetadata, StructMetadataList};

fn gather_base_fields(
    ast: &TypeDB,
    metalist: &StructMetadataList,
    root_bases: &[RecBindingDef],
) -> Vec<FieldMetadata> {
    let mut out = vec![];
    for base in root_bases {
        if let Some(b) = ast.recs.iter().find(|r| r.name() == base.name) {
            if let Some(base) = metalist.structs.iter().find(|s| s.name == base.name) {
                out.extend(base.fields.clone());
            }

            match b {
                RecBindingValue::Forward(_) => {}
                RecBindingValue::Def(r) => out.extend(gather_base_fields(ast, metalist, &r.bases)),
            }
        }
    }

    out
}

// HACKS to handle align etc.
const ALIGNED: &[&str] = &[
    "PxSListEntry",
    "PxSpringModifiers",
    "PxRestitutionModifiers",
    "PxContactPoint",
    "PxConstraintInvMassScale",
];

const PACKED: &[(&str, u8)] = &[
    ("PxControllerDesc", 4),
    ("PxOverlapHit", 0),
    ("PxSweepHit", 0),
    ("PxRaycastHit", 0),
];

impl crate::type_db::RecBindingDef {
    pub fn emit_odin(
        &self,
        w: &mut String,
        ast: &TypeDB,
        metalist: &StructMetadataList,
        level: u32,
    ) -> bool {
        if self.calc_layout {
            self.emit_odin_calc_layout(w, ast, metalist, level)
        } else {
            self.emit_odin_raw(w, level)
        }
    }

    pub fn emit_odin_calc_layout(
        &self,
        w: &mut String,
        ast: &TypeDB,
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

        let is_union = matches!(self.ast_tag_used, Some(crate::consumer::Tag::Union));
        let base_fields = gather_base_fields(ast, metalist, &self.bases);

        let align = if ALIGNED.contains(&self.name.as_str()) {
            "#align(16)"
        } else {
            ""
        };
        let (packed, free_bytes) = PACKED
            .iter()
            .find(|(n, _)| *n == self.name)
            .map(|(_, bytes)| ("#packed ", *bytes))
            .unwrap_or(("", 0u8));

        writesln!(
            w,
            "{indent}{} :: struct {}{align}{packed}{{",
            OdinIdent(&self.name),
            if is_union { "#raw_union " } else { "" },
        );

        for base in &self.bases {
            writesln!(w, "{indent1}using _: {},", OdinIdent(&base.name));
        }

        let prefix_pad = if self.bases.is_empty() {
            0
        } else {
            self.bases
                .iter()
                .filter_map(|base| metalist.structs.iter().find(|s| s.name == base.name))
                .map(|m| {
                    let strip_bytes = PACKED
                        .iter()
                        .find_map(|(n, b)| if *n == m.name { Some(*b) } else { None })
                        .unwrap_or(0);
                    m.size - strip_bytes as usize
                })
                .sum::<usize>()
        };

        for (idx, emitted_field) in meta.fields.iter().enumerate() {
            if emitted_field.name == "PAD" {
                if idx == 0 {
                    let width = if idx < meta.fields.len() - 1 {
                        emitted_field.size.saturating_sub(prefix_pad)
                    } else {
                        emitted_field
                            .size
                            .saturating_sub(prefix_pad)
                            .saturating_sub(free_bytes as usize)
                    };

                    if width > 0 {
                        writesln!(
                            w,
                            "{indent1}_pad{}: [{}]u8,",
                            base_fields.len() + idx,
                            width
                        );
                    }
                } else if idx < meta.fields.len() - 1 {
                    writesln!(
                        w,
                        "{indent1}_pad{}: [{}]u8,",
                        base_fields.len() + idx,
                        emitted_field.size
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}_pad{}: [{}]u8,",
                        base_fields.len() + idx,
                        emitted_field.size.checked_sub(free_bytes as usize).unwrap()
                    );
                }

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

            let prefix = if !field.is_public { "_private_" } else { "" };
            if let QualTypeValue::Array { element, len } = &field.kind {
                if let QualTypeValue::Array {
                    element: inner,
                    len: len1,
                } = &**element
                {
                    writesln!(
                        w,
                        "{indent1}{prefix}{}: [{}][{}]{},",
                        field.name,
                        len,
                        len1,
                        inner.odin_type(),
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}{prefix}{}: [{}]{},",
                        field.name,
                        len,
                        element.odin_type(),
                    );
                }
            } else {
                writesln!(
                    w,
                    "{indent1}{prefix}{}: {},",
                    field.name,
                    field.kind.odin_type()
                );
            }
        }

        writesln!(w, "{indent}}}\n");

        true
    }

    pub fn emit_odin_test(&self, w: &mut String, metalist: &StructMetadataList, level: u32) {
        if self.calc_layout {
            let meta = metalist
                .structs
                .iter()
                .find(|s| s.name == self.name)
                .unwrap();

            let indent1 = Indent(level + 1);

            let free_bytes = PACKED
                .iter()
                .find(|(n, _)| *n == self.name)
                .map(|(_, bytes)| *bytes)
                .unwrap_or(0u8);

            writesln!(w, "@(test)");
            writesln!(
                w,
                "test_layout_{} :: proc(t: ^testing.T) {{",
                OdinIdent(&self.name)
            );
            writesln!(w, "{indent1}using physx");

            for field in &meta.fields {
                if field.offset > 0 && field.name != "PAD" && !field.name.contains('[') {
                    writesln!(
						w,
						"{indent1}testing.expectf(t, offset_of({}, {}) == {}, \"Wrong offset for {}.{}, expected {} got %v\", offset_of({}, {}))",
						OdinIdent(&self.name),
						field.name,
						field.offset,
						OdinIdent(&self.name),
						field.name,
						field.offset,
						OdinIdent(&self.name),
						field.name,
					);
                }
            }
            writesln!(
				w,
				"{indent1}testing.expectf(t, size_of({}) == {}, \"Wrong size for type {}, expected {} got %v\", size_of({}))",
				OdinIdent(&self.name),
				meta.size as isize - free_bytes as isize,
				OdinIdent(&self.name),
				meta.size as isize - free_bytes as isize,
				 OdinIdent(&self.name),
			);
            writesln!(w, "}}");
        }
    }

    pub fn emit_odin_raw(&self, w: &mut String, level: u32) -> bool {
        let is_union = matches!(self.ast_tag_used, Some(crate::consumer::Tag::Union));

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(
            w,
            "{indent}{} :: struct {}{{",
            OdinIdent(&self.name),
            if is_union { "#raw_union " } else { "" },
        );

        if self.def_data.is_none() {
            return true;
        }

        for base in &self.bases {
            writesln!(w, "{indent1}using _: {},", OdinIdent(&base.name));
        }

        for field in &self.fields {
            if !field.is_public || field.is_reference {
                continue;
            }

            if let QualTypeValue::Array { element, len } = &field.kind {
                if let QualTypeValue::Array {
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

        writes!(w, "}};\n");
        true
    }
}

impl crate::type_db::RecBindingForward {
    pub fn emit_odin(&self, w: &mut String, level: u32) {
        let indent = Indent(level);

        writesln!(w, "{indent}{} :: distinct rawptr ", OdinIdent(&self.name));
    }
}
