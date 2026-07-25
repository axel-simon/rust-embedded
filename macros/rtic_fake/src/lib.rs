//! A host-only stand-in for `#[rtic::app]`. Recognizes the same
//! `#[shared]`/`#[local]`/`#[init]`/`#[idle]`/`#[task]` markers, matched by
//! attribute name only and discarding their arguments (`device =`,
//! `binds =`, `local =`, ...) — exactly how `rtic-macros` itself recognizes
//! them, since none of these are independently-resolvable macros in real
//! RTIC either. Instead of an interrupt-driven scheduler, it turns `init`
//! and each task into a plain, directly-callable function, so host unit
//! tests can drive them without any real concurrency.
//!
//! See `rtic_shim::app`, which picks this over real RTIC off-target.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{parse_macro_input, parse_quote, Item, ItemFn, ItemMod, ItemStruct};

#[proc_macro_attribute]
pub fn app(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let module = parse_macro_input!(item as ItemMod);
    expand(module).into()
}

fn expand(module: ItemMod) -> TokenStream2 {
    let ItemMod {
        attrs,
        vis,
        mod_token,
        ident,
        content,
        ..
    } = module;
    let (_brace, items) = content.expect("rtic_fake::app requires `mod app { ... }`, not `mod app;`");
    let items = items.into_iter().map(expand_item);

    quote! {
        #(#attrs)*
        #vis #mod_token #ident {
            #(#items)*
        }
    }
}

fn expand_item(item: Item) -> TokenStream2 {
    match item {
        Item::Struct(item_struct)
            if has_marker(&item_struct.attrs, "shared") || has_marker(&item_struct.attrs, "local") =>
        {
            expand_resources_struct(item_struct)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "init") => {
            expand_role(item_fn, &["init"], false)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "idle") => {
            expand_role(item_fn, &["idle"], true)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "task") => {
            expand_role(item_fn, &["task"], true)
        }
        other => quote!(#other),
    }
}

fn has_marker(attrs: &[syn::Attribute], name: &str) -> bool {
    attrs.iter().any(|attr| attr.path().is_ident(name))
}

fn strip_markers(attrs: Vec<syn::Attribute>, markers: &[&str]) -> Vec<syn::Attribute> {
    attrs
        .into_iter()
        .filter(|attr| !markers.iter().any(|marker| attr.path().is_ident(marker)))
        .collect()
}

/// `#[shared]`/`#[local]` structs pass through unchanged besides stripping
/// the marker attribute — their declared visibility (e.g. `pub(crate)`) is
/// what lets code outside `mod app` reach their fields, so it must survive
/// exactly as written, matching real RTIC's own behavior.
fn expand_resources_struct(mut item_struct: ItemStruct) -> TokenStream2 {
    item_struct.attrs = strip_markers(item_struct.attrs, &["shared", "local"]);
    quote!(#item_struct)
}

/// `#[init]`/`#[idle(...)]`/`#[task(...)]` functions become plain
/// `pub(crate)` functions (so a sibling `#[cfg(test)] mod tests` can call
/// them directly) alongside a matching `<name>::Context`. `Context` owns
/// `Local`/`Shared` by value rather than borrowing them — fake mode has no
/// real concurrency to protect, and it keeps the type non-generic so it
/// matches the plain `cx: <name>::Context` parameter written at the call
/// site (the same source RTIC's real macro also parses).
fn expand_role(mut item_fn: ItemFn, markers: &[&str], has_resources: bool) -> TokenStream2 {
    item_fn.attrs = strip_markers(item_fn.attrs, markers);
    item_fn.vis = parse_quote!(pub(crate));
    let name = &item_fn.sig.ident;
    let context = if has_resources {
        quote! {
            pub(crate) struct Context {
                pub(crate) local: super::Local,
                pub(crate) shared: super::Shared,
            }
        }
    } else {
        quote! {
            pub(crate) struct Context;
        }
    };

    quote! {
        #[allow(dead_code)]
        pub(crate) mod #name {
            #context
        }
        #item_fn
    }
}
