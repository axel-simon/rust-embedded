//! A host-only stand-in for `#[rtic::app]`. Recognizes the same
//! `#[shared]`/`#[local]`/`#[init]`/`#[idle]`/`#[task]` markers, matched by
//! attribute name only (arguments are otherwise discarded, exactly how
//! `rtic-macros` itself recognizes them, since none of these are
//! independently-resolvable macros in real RTIC either — the one
//! exception is `#[task(...)]`'s `binds = X` argument, which this crate
//! *does* read, to name the interrupt it registers with — see
//! [`generate_harness`]). Instead of an interrupt-driven scheduler, it
//! turns `init` and each task into a plain, directly-callable function, so
//! host unit tests can drive them without any real concurrency.
//!
//! Also recognizes `#[idle_step(...)]` — see [`expand_context_returning`]
//! for how it (and `#[task(...)]`) differ from a plain `#[idle]`.
//!
//! See `rtic_shim::app`, which picks this over real RTIC off-target.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::punctuated::Punctuated;
use syn::{
    parse_macro_input, parse_quote, Expr, FnArg, Item, ItemFn, ItemMod, ItemStruct, Meta, Pat,
    Token,
};

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
    let (_brace, items) =
        content.expect("rtic_fake::app requires `mod app { ... }`, not `mod app;`");

    // Collected while expanding each item below: every `#[task(binds = X,
    // ...)]`'s function name and the `X` it binds to — `generate_harness`
    // needs the full list up front, so it's gathered in one pass rather
    // than requiring a second traversal.
    let mut tasks = Vec::new();
    let items: Vec<TokenStream2> = items
        .into_iter()
        .map(|item| expand_item(item, &mut tasks))
        .collect();
    let harness = generate_harness(&tasks);

    quote! {
        #(#attrs)*
        #vis #mod_token #ident {
            #(#items)*
            #harness
        }
    }
}

fn expand_item(item: Item, tasks: &mut Vec<(syn::Ident, String)>) -> TokenStream2 {
    match item {
        Item::Struct(item_struct)
            if has_marker(&item_struct.attrs, "shared")
                || has_marker(&item_struct.attrs, "local") =>
        {
            expand_resources_struct(item_struct)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "init") => {
            expand_role(item_fn, &["init"], false)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "idle") => {
            expand_role(item_fn, &["idle"], true)
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "idle_step") => {
            expand_context_returning(item_fn, "idle_step")
        }
        Item::Fn(item_fn) if has_marker(&item_fn.attrs, "task") => {
            let binds = task_binds(&item_fn.attrs).unwrap_or_else(|| {
                panic!(
                    "#[task(...)] on `{}` needs a `binds = <interrupt>` argument -- rtic_fake \
                     doesn't yet support software (spawned) tasks",
                    item_fn.sig.ident
                )
            });
            tasks.push((item_fn.sig.ident.clone(), binds));
            expand_context_returning(item_fn, "task")
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

/// Extracts `binds`'s value out of a `#[task(binds = X, ...)]` attribute —
/// the only argument this crate reads (everything else, `priority =`/
/// `local = [...]`/`shared = [...]`, is discarded, same as before). `None`
/// if no `#[task(...)]` attribute is present, or it has no `binds = X`
/// (with `X` a plain identifier) among its arguments.
fn task_binds(attrs: &[syn::Attribute]) -> Option<String> {
    let attr = attrs.iter().find(|attr| attr.path().is_ident("task"))?;
    let metas = attr
        .parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)
        .ok()?;
    metas.into_iter().find_map(|meta| {
        let Meta::NameValue(name_value) = meta else {
            return None;
        };
        if !name_value.path.is_ident("binds") {
            return None;
        }
        let Expr::Path(expr_path) = name_value.value else {
            return None;
        };
        Some(expr_path.path.get_ident()?.to_string())
    })
}

/// `#[shared]`/`#[local]` structs pass through unchanged besides stripping
/// the marker attribute — their declared visibility (e.g. `pub(crate)`) is
/// what lets code outside `mod app` reach their fields, so it must survive
/// exactly as written, matching real RTIC's own behavior.
fn expand_resources_struct(mut item_struct: ItemStruct) -> TokenStream2 {
    item_struct.attrs = strip_markers(item_struct.attrs, &["shared", "local"]);
    quote!(#item_struct)
}

/// `#[init]`/`#[idle(...)]` functions become plain `pub(crate)` functions
/// (so a sibling `#[cfg(test)] mod tests` can call them directly) alongside
/// a matching `<name>::Context`. `Context` owns `Local`/`Shared` by value
/// rather than borrowing them — fake mode has no real concurrency to
/// protect, and it keeps the type non-generic so it matches the plain
/// `cx: <name>::Context` parameter written at the call site (the same
/// source RTIC's real macro also parses). Unlike
/// [`expand_context_returning`], the function's own return type/body are
/// untouched: `#[init]` must return `(Shared, Local)` (a hard requirement
/// from real RTIC too), and a plain `#[idle]` legitimately never returns
/// (`-> !`) — neither participates in the checkout/check-in convention
/// [`generate_harness`]'s registry-driven dispatch and
/// [`expand_context_returning`]'s functions share.
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

/// `#[idle_step(...)]`/`#[task(...)]` differ from a plain `#[idle(...)]`/
/// `#[init]` (`expand_role`, above) in exactly one way: the function
/// they're attached to describes a single call — no `loop { ... }`/
/// `-> !` of its own — rather than owning `Local`/`Shared` for the app's
/// whole lifetime. On real hardware, `rtic_real` wraps an `#[idle_step]`
/// in the `loop`/`-> !` real RTIC's own `#[idle]` requires (a real
/// `#[task]`'s binding needs no such rewrite, RTIC's own codegen handles
/// dispatch); here, there's no loop to add and no real interrupt to bind
/// — instead, this hands the `Context` it was given straight back as the
/// function's return value. Two things then rely on that:
///
/// - a caller (typically a host unit test) can thread an `#[idle_step]`
///   through repeated calls to simulate the real infinite loop one step at a
///   time: `let mut cx = ...; for _ in 0..n { cx = idle(cx); }`;
/// - [`generate_harness`]'s registry-driven dispatch for `#[task(...)]` relies
///   on the very same convention to check `Local`/`Shared` back in after
///   running a task it invoked on the app's behalf.
fn expand_context_returning(mut item_fn: ItemFn, marker: &str) -> TokenStream2 {
    let param = item_fn
        .sig
        .inputs
        .first()
        .unwrap_or_else(|| panic!("#[{marker}] function needs a Context parameter"));
    let FnArg::Typed(pat_type) = param else {
        panic!("#[{marker}] function can't take `self`");
    };
    let Pat::Ident(pat_ident) = pat_type.pat.as_ref() else {
        panic!("#[{marker}] function's Context parameter must be a plain binding (e.g. `cx`)");
    };
    let param_ident = pat_ident.ident.clone();

    item_fn.attrs = strip_markers(item_fn.attrs, &[marker]);
    item_fn.vis = parse_quote!(pub(crate));
    let name = &item_fn.sig.ident;
    item_fn.sig.output = parse_quote!(-> #name::Context);
    // `syn::Stmt`'s parser can't reliably tell a bare trailing expression
    // (no semicolon) apart from an incomplete statement when parsed in
    // isolation like this — going through `Expr` instead (unambiguous on
    // its own) and wrapping it in `Stmt::Expr(_, None)` (`None`: no
    // semicolon, i.e. this is the block's tail/return expression) sidesteps
    // that entirely.
    let tail_expr: Expr = parse_quote!(#param_ident);
    item_fn.block.stmts.push(syn::Stmt::Expr(tail_expr, None));

    quote! {
        #[allow(dead_code)]
        pub(crate) mod #name {
            pub(crate) struct Context {
                pub(crate) local: super::Local,
                pub(crate) shared: super::Shared,
            }
        }
        #item_fn
    }
}

/// Emits `Harness`, the fake backend's bridge between `init()`'s returned
/// `(Shared, Local)` and a `CallbackRegistry` (see
/// `peripherals::fake::callback_registry`): `Harness::new` wraps both in
/// shared, checkout-able cells and registers a closure for every
/// `#[task(binds = X, ...)]` in `tasks` under name `X`, so the registry
/// can autonomously invoke it (deciding *when*, per whatever it was
/// scheduled/fired for — see that crate) without a test needing to thread
/// `Context` through it by hand.
///
/// `Harness::checkout`/`checkin` are the *same* mechanism, exposed
/// directly, for whatever isn't a registry-driven task — typically an
/// `#[idle_step]`, which a test still drives manually. Both share the
/// same underlying cells as the registered tasks, which is what keeps
/// `#[shared]` fields consistent between the two: only one side (a
/// checked-out caller, or a firing task) can hold `Local`/`Shared` at a
/// time — see each panic message below.
///
/// `#[cfg(test)]`-gated: it needs `Rc`/`RefCell` (`std`), and firmware
/// crates in this workspace are `no_std` outside `cfg(test)` even on the
/// host target (see e.g. `firmware/benchtest/motor_control/src/main.rs`'s
/// `#![cfg_attr(not(test), no_std)]`) — this is purely test/simulation
/// infrastructure, never needed by a real (or even host, non-test) build.
fn generate_harness(tasks: &[(syn::Ident, String)]) -> TokenStream2 {
    let registrations = tasks.iter().map(|(name, binds)| {
        let checkout_panic = format!("{name} fired while Local/Shared was already checked out (a registry must only fire pending triggers between checkout/checkin cycles, never from inside one)");
        quote! {
            {
                let local = local_cell.clone();
                let shared = shared_cell.clone();
                registry.register(#binds, move |_now| {
                    let checked_out_local = local.borrow_mut().take().expect(#checkout_panic);
                    let checked_out_shared = shared.borrow_mut().take().expect(#checkout_panic);
                    let cx = #name(#name::Context {
                        local: checked_out_local,
                        shared: checked_out_shared,
                    });
                    *local.borrow_mut() = Some(cx.local);
                    *shared.borrow_mut() = Some(cx.shared);
                });
            }
        }
    });

    quote! {
        #[cfg(test)]
        #[allow(dead_code)]
        pub(crate) struct Harness {
            local: ::std::rc::Rc<::std::cell::RefCell<Option<Local>>>,
            shared: ::std::rc::Rc<::std::cell::RefCell<Option<Shared>>>,
        }

        #[cfg(test)]
        #[allow(dead_code)]
        impl Harness {
            pub(crate) fn new(
                local: Local,
                shared: Shared,
                registry: &::peripherals::fake::callback_registry::CallbackRegistry,
            ) -> Self {
                let local_cell = ::std::rc::Rc::new(::std::cell::RefCell::new(Some(local)));
                let shared_cell = ::std::rc::Rc::new(::std::cell::RefCell::new(Some(shared)));
                #(#registrations)*
                Harness {
                    local: local_cell,
                    shared: shared_cell,
                }
            }

            /// Checks `Local`/`Shared` out for direct use — e.g. to build
            /// an `#[idle_step]`'s `Context` by hand, exactly like
            /// `init()`'s own return value used to be used directly.
            /// Panics if already checked out (by a caller that hasn't
            /// called [`Self::checkin`] yet, or a registry-driven task
            /// currently running).
            pub(crate) fn checkout(&self) -> (Local, Shared) {
                let local = self.local.borrow_mut().take().expect(
                    "Harness::checkout() called while Local was already checked out",
                );
                let shared = self.shared.borrow_mut().take().expect(
                    "Harness::checkout() called while Shared was already checked out",
                );
                (local, shared)
            }

            /// Returns `Local`/`Shared` after a [`Self::checkout`] — see
            /// its doc comment.
            pub(crate) fn checkin(&self, local: Local, shared: Shared) {
                *self.local.borrow_mut() = Some(local);
                *self.shared.borrow_mut() = Some(shared);
            }
        }
    }
}
