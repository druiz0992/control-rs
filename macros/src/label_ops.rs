use proc_macro::TokenStream;
use quote::quote;
use syn::{Data, DeriveInput, Fields, parse_macro_input};

pub fn derive_label_ops_impl(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.ident;

    let Data::Struct(data_struct) = input.data else {
        panic!("Labelizable can only be derived for structs");
    };

    let Fields::Named(fields) = data_struct.fields else {
        panic!("Labelizable requires named fields");
    };

    // Collect label strings and their identifiers
    let mut labels = Vec::new();
    let mut idents = Vec::new();

    for field in fields.named.iter() {
        if let Some(ident) = &field.ident {
            labels.push(ident.to_string());
            idents.push(ident);
        }
    }

    // For `labels()`
    let label_literals: Vec<_> = labels.iter().map(|s| s.as_str()).collect();

    // For `index_of()`
    let match_arms = labels.iter().enumerate().map(|(i, label)| {
        quote! { #label => #i }
    });

    // For `from_labels()`
    let value_match_arms: Vec<_> = labels
        .iter()
        .zip(idents.iter())
        .map(|(label, ident)| {
            quote! {
                #label => self.#ident,
            }
        })
        .collect();

    let expanded = quote! {
        impl Labelizable for #name {
            fn labels() -> &'static [&'static str] {
                &[#(#label_literals),*]
            }

            fn index_of(label: &str) -> usize {
                match label {
                    #(#match_arms,)*
                    _ => panic!("Unknown label: {}", label),
                }
            }

            fn vectorize(&self, labels: &[&str]) -> Vec<f64> {
                labels.iter().map(|label| match *label {
                    #(#value_match_arms)*
                    _ => panic!("Unknown label: {}", label),
                }).collect()
            }

            fn extract<const N: usize>(&self, labels: &[&str]) -> [f64; N] {
                self.vectorize(labels)
                    .try_into()
                    .unwrap_or_else(|_| panic!("Expected exactly {} values for labels: {:?}", N, labels))
            }
        }
    };

    expanded.into()
}
