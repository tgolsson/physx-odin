use super::{Indent, SG, UOF};
use crate::consumer::{FieldBinding, QualType};
use crate::{writes, writesln};

impl<'ast> crate::consumer::RecBindingDef<'ast> {
    pub fn emit_c(&self, w: &mut String, level: u32) -> bool {
        if self.calc_layout {
            self.emit_c_definition(w, level)
        } else {
            self.emit_c_forward_decl(w, level)
        }
    }

    pub fn emit_c_definition(&self, w: &mut String, level: u32) -> bool {
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));
        writesln!(w, "{indent}// RecBindingDef::emit_c_definition");
        writesln!(
            w,
            "{indent}typedef {} physx_{} physx_{};",
            if is_union { "union" } else { "struct" },
            self.name,
            self.name
        );

        writesln!(
            w,
            "{indent}{} physx_{} {{",
            if is_union { "union" } else { "struct" },
            self.name
        );

        let mut count_fields = if self.has_vtable {
            writesln!(w, "{indent1}void* _vtable;");
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
                        "{indent1}{} {}[{}][{}];",
                        inner.real_ctype(),
                        field.name,
                        len1,
                        len
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}{} {}{};",
                        element.real_ctype(),
                        field.name,
                        len
                    );
                }
            } else {
                writesln!(w, "{indent1}{} {};", field.kind.real_ctype(), field.name);
            }
        }

        if count_fields == 0 {
            writesln!(w, "{indent1}char unused0[1];");
        }

        writesln!(w, "{indent}}};");
        true
    }

    pub fn emit_c_forward_decl(&self, w: &mut String, level: u32) -> bool {
        let is_union = matches!(self.ast.tag_used, Some(crate::consumer::Tag::Union));

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(w, "{indent}// RecBindingDef::emit_c_forward_decl");
        writesln!(
            w,
            "{indent}typedef {} physx_{} physx_{};",
            if is_union { "union" } else { "struct" },
            self.name,
            self.name
        );

        if self.ast.definition_data.is_none() {
            return true;
        }

        writes!(
            w,
            "{} physx_{}",
            if !is_union { "struct" } else { "union" },
            self.name
        );

        writes!(w, " {{\n");

        let mut count_fields = if self.has_vtable {
            writes!(w, "{indent1}void* vtable_;\n");
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
                        "{indent1}{} {}[{}][{}];",
                        field.name,
                        inner.real_ctype(),
                        len1,
                        len
                    );
                } else {
                    writesln!(
                        w,
                        "{indent1}{} {}{};",
                        element.real_ctype(),
                        field.name,
                        len
                    );
                }
            } else {
                writesln!(w, "{indent1}{} {};", field.kind.real_ctype(), field.name);
            }
        }

        if count_fields == 0 {
            writesln!(w, "{indent1}char _unused0[1];");
        }

        writes!(w, "}};\n");
        true
    }
}

impl<'ast> crate::consumer::RecBindingForward<'ast> {
    pub fn emit_c(&self, w: &mut String, level: u32) {
        let indent = Indent(level);

        writesln!(w, "{indent}// RecBindingForward::emit_c");
        writesln!(
            w,
            "{indent}typedef struct physx_{} physx_{};",
            self.name,
            self.name
        );
    }
}
