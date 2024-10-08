use super::Indent;
use crate::consumer::EnumBinding;
use crate::odin_generator::OdinIdent;
use crate::{writes, writesln};

/// Fixes enum variant names from the ugly C++ style of `eWHY_ARE_YOU_SHOUTING`
/// to `WhyAreYouShouting`
fn fix_variant_name(s: &str) -> String {
    let no_e = s
        .strip_prefix('e')
        .filter(|s| s.chars().next().unwrap().is_ascii_alphabetic())
        .unwrap_or(s);

	let no_px = no_e.strip_prefix("Px")
		.unwrap_or(no_e);
	let no_px = no_e.strip_prefix("PX")
		.unwrap_or(no_px);

    use heck::ToUpperCamelCase;
    no_px.to_upper_camel_case()
}

impl<'ast> EnumBinding<'ast> {
    pub fn emit_odin(&self, w: &mut String, is_flags: bool, level: u32) {
        if is_flags {
            self.emit_as_flags(w, level)
        } else {
            self.emit_as_enum(w, level)
        }
    }

    pub fn emit_as_flags(&self, w: &mut String, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_odin(w, level);
        }

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(w, "{indent}{} :: enum {} {{", OdinIdent(self.name), self.repr.odin_type());

        for var in &self.variants {
            // Since bitflags are made up of power of 2 values that can
            // be combined, and the PhysX API sometimes defines named
            // combinations of flags, reconstruct the bitflags to be
            // easier to read
            let val = var.value as u64;
            if val == 0 {
                continue;
            } else if val & (val - 1) == 0 {
                writesln!(
                    w,
                    "{indent1}{} = {},",
                    fix_variant_name(var.name),
                    val.ilog2()
                );
            }
        }

        writesln!(w, "}}");

        for var in &self.variants {
            // Since bitflags are made up of power of 2 values that can
            // be combined, and the PhysX API sometimes defines named
            // combinations of flags, reconstruct the bitflags to be
            // easier to read
            let val = var.value as u64;
            if val != 0 && (val & (val - 1)) == 0 {
                continue;
            }

            if val == 0 {
                writes!(
                    w,
                    "{indent}{}_{} :: 0",
                    OdinIdent(self.name),
                    fix_variant_name(var.name)
                );
            } else {
                writes!(
                    w,
                    "{indent}{}_{} :: ",
                    OdinIdent(self.name),
                    fix_variant_name(var.name)
                );
                let mut is_combo = false;
                for (i, which) in self
                    .variants
                    .iter()
                    .filter_map(|var| {
                        let prev = var.value as u64;
                        (prev > 0 && (prev & (prev - 1) == 0 && (prev & val) != 0))
                            .then_some(var.name)
                    })
                    .enumerate()
                {
                    is_combo = true;
                    if i > 0 {
                        writes!(w, " | ");
                    }

                    writes!(w, "{}.{}", OdinIdent(self.name), fix_variant_name(which));
                }

                if !is_combo {
                    writes!(w, "0x{val:08x}");
                }
            }
            writesln!(w, "")
        }
    }

    pub fn emit_as_enum(&self, w: &mut String, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_odin(w, level);
        }

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(w, "{indent}{} :: enum {} {{", OdinIdent(self.name), self.repr.odin_type());

        for var in &self.variants {
            if let Some(com) = &var.comment {
                com.emit_odin(w, level + 1);
            }


            writesln!(
                w,
                "{indent1}{} = {},",
                fix_variant_name(var.name),
                var.value
            );
        }
        writesln!(w, "}}");
    }
}

impl<'ast> crate::consumer::FlagsBinding<'ast> {
    pub fn emit_odin(&self, enum_binding: &EnumBinding<'ast>, w: &mut String, level: u32) {
        let indent = Indent(level);

        writesln!(w, "{indent}/// Flags for [`{}`]", enum_binding.name);

        writesln!(
            w,
            "{indent}{}_Set :: bit_set[{}; {}]\n",
            OdinIdent(self.name),
            OdinIdent(enum_binding.name),
            self.storage_type.odin_type()
        );
    }
}
