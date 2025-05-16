use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

pub fn derive_label_ops_impl(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let data = match input.data {
        syn::Data::Struct(data) => data,
        _ => panic!("ArrayLabelOps can only be derived for structs"),
    };

    let fields = match data.fields {
        syn::Fields::Named(ref fields) => &fields.named,
        _ => panic!("ArrayLabelOps requires named fields"),
    };

    if fields.len() != 1 {
        panic!("Expected exactly one field");
    }

    let field = fields.iter().next().unwrap();
    let field_ident = field.ident.clone().unwrap();

    // Prefix: lowercase of struct name
    let prefix = name.to_string().to_lowercase();

    let expanded = quote! {
        impl #impl_generics Labelizable for #name #ty_generics #where_clause {
            fn labels() -> &'static [&'static str] {
                // SAFETY: Leaked strings live forever.
                static mut LABELS: Option<Vec<&'static str>> = None;
                static INIT: std::sync::Once = std::sync::Once::new();

                INIT.call_once(|| {
                    let labels: Vec<&'static str> = (0..N)
                        .map(|i| {
                            let s = format!("{}_{}", #prefix, i);
                            Box::leak(s.into_boxed_str()) as &'static str
                        })
                        .collect();
                    unsafe {
                        LABELS = Some(labels);
                    }
                });

                unsafe {
                    LABELS.as_ref().unwrap()
                }
            }

            fn index_of(label: &str) -> usize {
                Self::labels()
                    .iter()
                    .position(|&l| l == label)
                    .unwrap_or_else(|| panic!("Unknown label: {}", label))
            }

            fn vectorize(&self, labels: &[&str]) -> Vec<f64> {
                labels.iter().map(|&label| {
                    let i = Self::index_of(label);
                    self.#field_ident[i]
                }).collect()
            }

            fn extract<const K: usize>(&self, labels: &[&str]) -> [f64; K] {
                self.vectorize(labels)
                    .try_into()
                    .unwrap_or_else(|_| panic!("Expected exactly {} values", K))
            }
        }
    };

    expanded.into()
}
