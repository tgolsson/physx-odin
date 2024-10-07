use super::Indent;
use crate::consumer::{self, QualType, RecBindingDef};
use crate::{writes, writesln, FieldMetadata, StructMetadataList};

fn gather_base_fields<'a, 'ast>(
    ast: &consumer::AstConsumer<'ast>,
    metalist: &'a StructMetadataList,
    root_bases: &[RecBindingDef<'ast>],
) -> Vec<FieldMetadata> {
    let mut out = vec![];
    for base in root_bases {
        if let Some(b) = ast.recs.iter().find(|r| r.name() == base.name) {
            if let Some(base) = metalist.structs.iter().find(|s| s.name == base.name) {
                out.extend(base.fields.clone());
            }

            match b {
                consumer::RecBinding::Forward(_) => {}
                consumer::RecBinding::Def(r) => {
                    out.extend(gather_base_fields(ast, metalist, &r.bases))
                }
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
    // ("PxTGSSolverConstraintPrepDescBase", 8),
	// ("PxSolverConstraintPrepDescBase", 8),
];

const REAR_PAD: &[(&str, u8)] = &[

    ("PxSolverConstraintPrepDescBase", 8),
//	("PxSolverContactDesc", 8),
];

impl<'ast> crate::consumer::RecBindingDef<'ast> {
    pub fn emit_odin(
        &self,
        w: &mut String,
        ast: &consumer::AstConsumer<'ast>,
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
        ast: &consumer::AstConsumer<'ast>,
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
        let base_fields = gather_base_fields(ast, metalist, &self.bases);

        let align = if ALIGNED.contains(&self.name) {
            "#align(16)"
        } else {
            ""
        };
        let (packed, free_bytes) = PACKED
            .iter()
            .find(|(n, _)| *n == self.name)
            .map(|(_, bytes)| ("#packed ", *bytes))
            .unwrap_or(("", 0u8));

        let base_has_vtable = self.bases.iter().any(|f| f.has_vtable);
        writesln!(
            w,
            "{indent}{} :: struct {}{align}{packed}{{",
            self.name,
            if is_union { "#raw_union " } else { "" },
        );

        writesln!(w, "// {:?} {:?}", self.has_vtable, base_has_vtable);
        writesln!(
            w,
            "// {:?}",
            self.bases.iter().map(|b| b.name).collect::<Vec<_>>()
        );
        if self.has_vtable && !base_has_vtable {
            writesln!(w, "{indent1}vtable: rawptr,");
        }

        for base in &self.bases {
            writesln!(w, "{indent1}using _: {},", base.name);
        }

        let mut count_fields = 0;
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

        writesln!(w, "// {prefix_pad}");
        for (idx, emitted_field) in meta.fields.iter().enumerate() {
            if emitted_field.name == "PAD" {
                if idx == 0 {
                    if prefix_pad < emitted_field.size {
                        writesln!(w, "// XXXX {} < {}", emitted_field.size, prefix_pad);
                    }
                    writesln!(
                        w,
                        "{indent1}_pad{}: [{}]u8,",
                        base_fields.len() + idx,
                        prefix_pad .saturating_sub(emitted_field.size)
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}_pad{}: [{}]u8,",
                        base_fields.len() + idx,
                        emitted_field.size
                    );
                }

                count_fields += 1;
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

            count_fields += 1;
            let prefix = if !field.is_public { "_private_" } else { "" };
            if let QualType::Array { element, len } = &field.kind {
                if let QualType::Array {
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

		if let Some(trailing_bytes) = REAR_PAD.iter().find_map(|(n,  b)| (*n == self.name).then_some(*b)) {
			writesln!(w, "{indent1}_size_fix: [{trailing_bytes}]u8,");
		}
        if count_fields == 0 && self.bases.is_empty() {
            writesln!(w, "{indent1}unused0: [1]u8,");
        }

        writesln!(w, "{indent}}}");

        writesln!(w, "@(test)");
        writesln!(w, "test_layout_{} :: proc(t: ^testing.T) {{", self.name);

        for field in &meta.fields {
            if field.offset > 0 && field.name != "PAD" && !field.name.contains('[') {
                writesln!(
					w,
					"{indent1}testing.expectf(t, offset_of({}, {}) == {}, \"Wrong offset for {}.{}, expected {} got %v\", offset_of({}, {}))",
					self.name,
					field.name,
					field.offset,
					self.name,
					field.name,
					field.offset,
					self.name,
					field.name,
				);
            }
        }
        writesln!(
            w,
			"{indent1}testing.expectf(t, size_of({}) == {}, \"Wrong size for type {}, expected {} got %v\", size_of({}))",
            self.name,
            meta.size as isize - free_bytes as isize,
            self.name,
            meta.size as isize - free_bytes as isize,
			self.name,
        );
        writesln!(w, "}}");
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
