use super::Indent;
use crate::consumer::{functions::*, AstConsumer, QualType};
use crate::{writes, writesln};

fn cleanup_name(name: &str) -> String {
    let type_name = if name.starts_with("phys_Px") {
        name.strip_prefix("phys_Px")
    } else if name.starts_with("physx_Px") {
        name.strip_prefix("physx_Px")
    } else if name.starts_with("Px") {
        name.strip_prefix("Px")
    } else if name.starts_with("Interpolation_Px") {
        name.strip_prefix("Interpolation_Px")
    } else {
        name.strip_prefix("phys_")
    }
    .unwrap();

    let mut out = String::with_capacity(type_name.len() + 10); // some extra cap

    for (idx, c) in type_name.chars().enumerate() {
        if c.is_uppercase() {
            if idx > 0 {
                out.push('_');
            }
            out.extend(c.to_lowercase());
        } else {
            out.push(c);
        }
    }

	if out.contains("c_c_d")  {
		out = out.replace("c_c_d", "ccd");
	}

	if out == "log" {
		out = "quat_log".to_owned();
	}

    out
}

impl<'ast> FuncBinding<'ast> {
    pub(super) fn emit_odin(&self, writer: &mut String, ast: &AstConsumer<'_>, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_odin(writer, level);
        }

        let indent = Indent(level);
        let mut acc = String::new();

        writesln!(acc, "{indent}@(link_name = \"{}\")", self.name);
        writes!(acc, "{indent}{} :: proc(", cleanup_name(&self.name));


        for (i, param) in self.params.iter().enumerate() {
			let has_derived =
				match &param.kind {
					QualType::Reference { pointee, .. } => {
						match &**pointee {
							QualType::Record { name } => {
								ast.derived.get(*name).map(|v| !v.is_empty()).unwrap_or(false)
							}
							_ => false
						}

					}
					_=> false,
				};
            let sep = if i > 0 { ", " } else { "" };
            let typ = param.kind.odin_type();
			let tag = if typ.is_const_ref() && !has_derived {
				"#by_ptr "
			} else {
				""
			};
            writes!(
                acc,
                "{sep}{}{}: {}",
				tag,
                super::OdinIdent(&param.name),
                if typ.is_const_ref() && !has_derived { typ.const_ref_type() } else { typ },
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
