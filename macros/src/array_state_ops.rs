use proc_macro::TokenStream;
use quote::quote;
use syn::{DeriveInput, parse_macro_input};

pub fn derive_state_ops_impl(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let fields = match &input.data {
        syn::Data::Struct(syn::DataStruct {
            fields: syn::Fields::Named(fields),
            ..
        }) => &fields.named,
        _ => panic!("Only named structs are supported"),
    };

    if fields.len() != 1 {
        panic!("Expected exactly one field");
    }

    let field = fields.iter().next().unwrap();
    let field_ident = field.ident.as_ref().unwrap();

    let expanded = quote! {
        impl #impl_generics #name #ty_generics #where_clause {
            pub fn new(a: [f64; N]) -> Self {
                assert!(C <= N, "C must be <= N");
                Self { #field_ident: a }
            }
        }

        impl #impl_generics State for #name #ty_generics #where_clause {
            fn to_vec(&self) -> Vec<f64> {
                self.#field_ident.to_vec()
            }

            fn from_vec(v: Vec<f64>) -> Self {
                assert_eq!(v.len(), N, "Expected vector of length N");
                let mut a = [0.0; N];
                a.copy_from_slice(&v);
                Self { #field_ident: a }
            }

            fn from_slice(v: &[f64]) -> Self {
                assert_eq!(v.len(), N, "Expected slice of length N");
                let mut a = [0.0; N];
                a.copy_from_slice(v);
                Self { #field_ident: a }
            }

            fn get_q(&self) -> Vec<f64> {
                self.#field_ident[0..(N - C)].to_vec()
            }

            fn get_v(&self) -> Vec<f64> {
                self.#field_ident[(N - C)..N].to_vec()
            }

            fn dim_q() -> usize {
                N - C
            }

            fn dim_v() -> usize {
                C
            }
        }

        impl #impl_generics std::ops::Add for #name #ty_generics #where_clause {
            type Output = Self;
            fn add(self, rhs: Self) -> Self::Output {
                let mut res = [0.0; N];
                for i in 0..N {
                    res[i] = self.#field_ident[i] + rhs.#field_ident[i];
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Sub for #name #ty_generics #where_clause {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                let mut res = [0.0; N];
                for i in 0..N {
                    res[i] = self.#field_ident[i] - rhs.#field_ident[i];
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Mul<f64> for #name #ty_generics #where_clause {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self::Output {
                let mut res = [0.0; N];
                for i in 0..N {
                    res[i] = self.#field_ident[i] * rhs;
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Div<f64> for #name #ty_generics #where_clause {
            type Output = Self;
            fn div(self, rhs: f64) -> Self::Output {
                let mut res = [0.0; N];
                for i in 0..N {
                    res[i] = self.#field_ident[i] / rhs;
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics PartialEq for #name #ty_generics #where_clause {
            fn eq(&self, other: &Self) -> bool {
                self.#field_ident
                    .iter()
                    .zip(other.#field_ident.iter())
                    .all(|(a, b)| a == b)
            }
        }
    };

    expanded.into()
}
