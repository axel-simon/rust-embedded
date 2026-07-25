//! Wraps real RTIC's `#[rtic::app]`, forcing `peripherals = false` — this
//! workspace always brings the chip up through a board crate's own
//! `initialize()` (`embassy_stm32::init()` under the hood), never through
//! RTIC's own PAC-peripherals claim, so callers shouldn't need to repeat
//! that argument (or be able to accidentally override it) at every call
//! site. Just emits `#[rtic::app(<given args>, peripherals = false)]`
//! ahead of the item unchanged — `rtic` itself only needs to be a
//! dependency of whichever crate this expands into, not of this crate.
//!
//! See `rtic_shim::app`, which picks this over the host-only fake.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;

#[proc_macro_attribute]
pub fn app(attr: TokenStream, item: TokenStream) -> TokenStream {
    let attr = TokenStream2::from(attr);
    let item = TokenStream2::from(item);

    quote! {
        #[rtic::app(#attr, peripherals = false)]
        #item
    }
    .into()
}
