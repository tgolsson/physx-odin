use crate::consumer::{Builtin, EnumBinding};
use crate::{writesln, writes};
use super::Indent;

/// Fixes enum variant names from the ugly C++ style of `eWHY_ARE_YOU_SHOUTING`
/// to `WhyAreYouShouting`
fn fix_variant_name(s: &str) -> String {
    let no_e = s
        .strip_prefix('e')
        .filter(|s| s.chars().next().unwrap().is_ascii_alphabetic())
        .unwrap_or(s);

    use heck::ToUpperCamelCase;
    no_e.to_upper_camel_case()
}

impl<'ast> EnumBinding<'ast> {
    pub fn emit_c(&self, w: &mut String, level: u32) {
        if let Some(com) = &self.comment {
            com.emit_c(w, level);
        }

        let indent = Indent(level);
        let indent1 = Indent(level + 1);
        writesln!(w, "{indent}typedef enum {}: {} {{", self.name, self.repr.c_type());

        for var in &self.variants {
            if let Some(com) = &var.comment {
                com.emit_c(w, level + 1);
            }

            writesln!(
                w,
                "{indent1}{}_{} = {},",
				self.name,
                fix_variant_name(var.name),
                var.value
            );
        }

        writesln!(w, "{indent}}} {};", self.name);
    }

    pub fn emit_c_conversion(&self, w: &mut String, level: u32, from: Builtin, default: &str) {

    }
}

impl<'ast> crate::consumer::FlagsBinding<'ast> {
    pub fn emit_c(&self, enum_binding: &EnumBinding<'ast>, w: &mut String, level: u32) {
        let indent = Indent(level);
        let indent1 = Indent(level + 1);

		return;


        writesln!(w, "{indent}/// Flags for [`{}`]", enum_binding.name);

        writesln!(
            w,
            "{indent}typedef enum {}: {} {{",
            self.name,
            self.storage_type.real_ctype()
        );

        for var in &enum_binding.variants {
            writes!(w, "{indent1}{}_{} = ", self.name, fix_variant_name(var.name));

            // Since bitflags are made up of power of 2 values that can
            // be combined, and the PhysX API sometimes defines named
            // combinations of flags, reconstruct the bitflags to be
            // easier to read
            let val = var.value as u64;
			if val == 0 {
				writes!(w, "0 << 0");
			}
            else if val & (val - 1) == 0 {
                writes!(w, "1 << {}", val.ilog2());
            } else {
                let mut is_combo = false;
                // If we're not a power of 2, we're a combination of flags,
                // find which ones and emit them in a friendly way

                for (i, which) in enum_binding
                    .variants
                    .iter()
                    .filter_map(|var| {
                        let prev = var.value as u64;
                        (prev > 0 && (prev & (prev - 1) == 0 && (prev & val) != 0)).then_some(var.name)
                    })
                    .enumerate()
                {
                    is_combo = true;
                    if i > 0 {
                        writes!(w, " | ");
                    }

                    writes!(w, "{}_{}", self.name, fix_variant_name(which));
                }

                // There are a couple of cases where they're not combos, so just
                // emit the raw value
                if !is_combo {
                    writes!(w, "0x{val:08x}");
                }
            }

            writesln!(w, ",");
        }

        writesln!(w, "{indent}}} {};\n", self.name);
    }
}
