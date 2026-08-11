//! Wraps real RTIC's `#[rtic::app]`, forcing `peripherals = false` — this
//! workspace always brings the chip up through a board crate's own
//! `initialize()` (`embassy_stm32::init()` under the hood), never through
//! RTIC's own PAC-peripherals claim, so callers shouldn't need to repeat
//! that argument (or be able to accidentally override it) at every call
//! site.
//!
//! Also rewrites an `#[idle_step(...)]`-marked function (see
//! [`rewrite_idle_step`]) into a proper `#[idle(...)]` task before handing
//! the module to real RTIC — everything else passes through unchanged,
//! since `rtic` itself only needs to be a dependency of whichever crate
//! this expands into, not of this crate.
//!
//! See `rtic_shim::app`, which picks this over the host-only fake.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{parse_macro_input, parse_quote, Attribute, Item, ItemMod, Meta};

#[proc_macro_attribute]
pub fn app(attr: TokenStream, item: TokenStream) -> TokenStream {
    let attr = TokenStream2::from(attr);
    let mut module = parse_macro_input!(item as ItemMod);
    rewrite_idle_step(&mut module);

    quote! {
        #[rtic::app(#attr, peripherals = false)]
        #module
    }
    .into()
}

/// Finds the (at most one — real RTIC only allows one `#[idle]` anyway)
/// function marked `#[idle_step(...)]` and turns it into what real RTIC's
/// own `#[idle(...)]` requires: `fn idle(mut cx: idle::Context) -> !`,
/// running forever. A function written for `#[idle_step]` describes only
/// a single iteration (see `rtic_fake`'s matching support for why — it
/// lets host unit tests call that single iteration directly and
/// repeatedly instead), so this renames the marker to `idle` (keeping
/// whatever arguments it was given, e.g. `local = [...]`/`shared =
/// [...]`, untouched) and wraps its body in a `loop`.
fn rewrite_idle_step(module: &mut ItemMod) {
    let Some((_, items)) = &mut module.content else {
        return;
    };
    for item in items {
        let Item::Fn(item_fn) = item else { continue };
        let Some(index) = item_fn
            .attrs
            .iter()
            .position(|attr| attr.path().is_ident("idle_step"))
        else {
            continue;
        };

        let attr = item_fn.attrs.remove(index);
        let idle_attr: Attribute = match attr.meta {
            Meta::List(list) => {
                let args = list.tokens;
                parse_quote!(#[idle(#args)])
            }
            Meta::Path(_) => parse_quote!(#[idle]),
            Meta::NameValue(_) => {
                panic!("#[idle_step = ...] isn't supported — use #[idle_step(...)]")
            }
        };
        item_fn.attrs.insert(index, idle_attr);

        let body = item_fn.block.clone();
        item_fn.block = parse_quote!({ loop #body });
        item_fn.sig.output = parse_quote!(-> !);
    }
}
