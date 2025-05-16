extern crate proc_macro;
use proc_macro::TokenStream;

mod array_label_ops;
mod array_state_ops;
mod label_ops;
mod state_ops;

#[proc_macro_derive(StateOps, attributes(constrained))]
pub fn derive_state_ops(item: TokenStream) -> TokenStream {
    state_ops::derive_state_ops_impl(item)
}

#[proc_macro_derive(ArrayStateOps)]
pub fn derive_array_state_ops(item: TokenStream) -> TokenStream {
    array_state_ops::derive_state_ops_impl(item)
}

#[proc_macro_derive(LabelOps)]
pub fn derive_label_ops(item: TokenStream) -> TokenStream {
    label_ops::derive_label_ops_impl(item)
}

#[proc_macro_derive(ArrayLabelOps)]
pub fn derive_array_label_ops(item: TokenStream) -> TokenStream {
    array_label_ops::derive_label_ops_impl(item)
}
