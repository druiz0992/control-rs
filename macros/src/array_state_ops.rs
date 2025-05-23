use proc_macro::TokenStream;
use quote::quote;
use syn::{ConstParam, DeriveInput, GenericParam, parse_macro_input};

pub fn derive_state_ops_impl(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    // Extract generics info
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    // Find the first const generic identifier (e.g. I)
    let first_const_generic_ident = generics
        .params
        .iter()
        .find_map(|param| {
            if let GenericParam::Const(ConstParam { ident, .. }) = param {
                Some(ident)
            } else {
                None
            }
        })
        .expect("Expected at least one const generic parameter");

    // Get the fields (named structs only)
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
            pub fn new(a: [f64; #first_const_generic_ident]) -> Self {
                assert!(C <= #first_const_generic_ident, "C must be <= N");
                Self { #field_ident: a }
            }

        }


        impl #impl_generics State for #name #ty_generics #where_clause {
            fn to_vec(&self) -> Vec<f64> {
                self.#field_ident.to_vec()
            }

            fn to_vector(&self) -> ::nalgebra::DVector<f64> {
                ::nalgebra::DVector::from_iterator(#first_const_generic_ident, self.#field_ident.iter().copied())
            }

            fn from_vector(vec: ::nalgebra::DVector<f64>) -> Self {
                assert_eq!(vec.len(), #first_const_generic_ident, "Expected vector of length {}", #first_const_generic_ident);
                let mut a = [0.0; #first_const_generic_ident];
                a.copy_from_slice(vec.as_slice());
                Self { #field_ident: a }
            }

            fn from_vec(v: Vec<f64>) -> Self {
                assert_eq!(v.len(), #first_const_generic_ident, "Expected vector of length N");
                let mut a = [0.0; #first_const_generic_ident];
                a.copy_from_slice(&v);
                Self { #field_ident: a }
            }

            fn from_slice(v: &[f64]) -> Self {
                assert_eq!(v.len(), #first_const_generic_ident, "Expected slice of length N");
                let mut a = [0.0; #first_const_generic_ident];
                a.copy_from_slice(v);
                Self { #field_ident: a }
            }

            fn get_q(&self) -> Vec<f64> {
                self.#field_ident[0..(#first_const_generic_ident - C)].to_vec()
            }

            fn get_v(&self) -> Vec<f64> {
                self.#field_ident[(#first_const_generic_ident - C)..#first_const_generic_ident].to_vec()
            }

            fn dim_q() -> usize {
                #first_const_generic_ident - C
            }

            fn dim_v() -> usize {
                C
            }
        }

        impl #impl_generics Default for #name #ty_generics #where_clause {
            fn default() -> Self {
                Self {
                    #field_ident: [0.0; #first_const_generic_ident],
                }
            }
        }

        impl #impl_generics std::ops::Add for #name #ty_generics #where_clause {
            type Output = Self;
            fn add(self, rhs: Self) -> Self::Output {
                let mut res = [0.0; #first_const_generic_ident];
                for i in 0..#first_const_generic_ident {
                    res[i] = self.#field_ident[i] + rhs.#field_ident[i];
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Sub for #name #ty_generics #where_clause {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                let mut res = [0.0; #first_const_generic_ident];
                for i in 0..#first_const_generic_ident {
                    res[i] = self.#field_ident[i] - rhs.#field_ident[i];
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Mul<f64> for #name #ty_generics #where_clause {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self::Output {
                let mut res = [0.0; #first_const_generic_ident];
                for i in 0..#first_const_generic_ident {
                    res[i] = self.#field_ident[i] * rhs;
                }
                Self { #field_ident: res }
            }
        }

        impl #impl_generics std::ops::Div<f64> for #name #ty_generics #where_clause {
            type Output = Self;
            fn div(self, rhs: f64) -> Self::Output {
                let mut res = [0.0; #first_const_generic_ident];
                for i in 0..#first_const_generic_ident {
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
