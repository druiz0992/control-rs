use proc_macro::TokenStream;
use quote::quote;
use syn::{DeriveInput, parse_macro_input};

/// This procedural macro derives the `StateOps` trait for a struct with named fields.
///
/// The macro generates the following implementations and methods:
///
/// 1. **Constructor Method**:
///    - `pub fn new(...) -> Self`
///    - Creates a new instance of the struct by accepting one `f64` argument for each field.
///
/// 2. **State Method**:
///    - `pub fn state(&self) -> (f64, f64, ...)`
///    - Returns a tuple containing the values of all fields in the struct.
///
/// 3. **State Trait Implementation**:
///    - Implements the `State` trait with the following methods:
///      - `fn to_vec(&self) -> Vec<f64>`: Converts the struct's fields into a `Vec<f64>`.
///      - `fn from_vec(v: Vec<f64>) -> Self`: Constructs the struct from a `Vec<f64>`.
///        Ensures the vector has the correct number of elements.
///      - `fn labels() -> &'static [&'static str]`: Returns the names of the struct's fields as a slice of strings.
///
/// 4. **Arithmetic Operations**:
///    - Implements the following operator traits for the struct:
///      - `std::ops::Add`: Adds corresponding fields of two instances.
///      - `std::ops::Sub`: Subtracts corresponding fields of two instances.
///      - `std::ops::Mul<f64>`: Multiplies each field by a scalar value.
///      - `std::ops::Div<f64>`: Divides each field by a scalar value.
///
/// 5. **Equality Comparison**:
///    - Implements `PartialEq` for the struct:
///      - Compares two instances by checking if all corresponding fields are equal.
///
/// ## Restrictions:
/// - The macro only supports structs with named fields.
/// - All fields must be of type `f64`.
///
pub fn derive_state_ops_impl(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let fields = match &input.data {
        syn::Data::Struct(syn::DataStruct {
            fields: syn::Fields::Named(fields),
            ..
        }) => &fields.named,
        _ => panic!("Only named structs are supported"),
    };

    let field_names: Vec<_> = fields.iter().map(|f| f.ident.as_ref().unwrap()).collect();
    let num_fields = field_names.len();

    let (q_fields, v_fields): (Vec<_>, Vec<_>) = fields.iter().partition(|f| {
        f.attrs
            .iter()
            .all(|attr| !attr.path().is_ident("constrained"))
    });

    let q_idents: Vec<_> = q_fields.iter().map(|f| f.ident.as_ref().unwrap()).collect();
    let v_idents: Vec<_> = v_fields.iter().map(|f| f.ident.as_ref().unwrap()).collect();

    // For `new` method
    let new_fn_args = field_names.iter().map(|f| quote! { #f: f64 });
    let new_fn_init = field_names.iter().map(|f| quote! { #f });

    // For State trait
    let to_vec_fields = field_names.iter().map(|f| quote! { self.#f });
    let from_vec_fields_vec: Vec<_> = field_names
        .iter()
        .enumerate()
        .map(|(i, f)| quote! { #f: v[#i] })
        .collect();

    let from_vec_fields_slice = from_vec_fields_vec.clone(); // requires Clone

    // For ops
    let add_fields = field_names.iter().map(|f| quote! { #f: self.#f + rhs.#f });
    let sub_fields = field_names.iter().map(|f| quote! { #f: self.#f - rhs.#f });
    let mul_fields = field_names.iter().map(|f| quote! { #f: self.#f * rhs });
    let div_fields = field_names.iter().map(|f| quote! { #f: self.#f / rhs });
    let eq_fields = field_names.iter().map(|f| quote! { self.#f == other.#f });

    let q_len = q_idents.len();
    let v_len = v_idents.len();

    let expanded = quote! {
        impl #name {
            pub fn new( #( #new_fn_args ),* ) -> Self {
                Self { #( #new_fn_init ),* }
            }
        }

        impl State for #name {
            fn to_vec(&self) -> Vec<f64> {
                vec![ #( #to_vec_fields ),* ]
            }


            fn from_vec(v: Vec<f64>) -> Self {
                assert_eq!(v.len(), #num_fields, "Expected {} elements", #num_fields);
                Self { #( #from_vec_fields_vec ),* }
            }

            fn from_slice(v: &[f64]) -> Self {
                assert_eq!(v.len(), #num_fields, "Expected {} elements", #num_fields);
                Self { #( #from_vec_fields_slice ),* }
            }

            fn get_q(&self) -> Vec<f64> {
                vec![ #( self.#q_idents ),* ]
            }

            fn get_v(&self) -> Vec<f64> {
                vec![ #( self.#v_idents ),* ]
            }

            fn dim_q() -> usize {
                #q_len
            }

            fn dim_v() -> usize {
                #v_len
            }
        }

        impl std::ops::Add for #name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self::Output {
                Self { #( #add_fields ),* }
            }
        }

        impl std::ops::Sub for #name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                Self { #( #sub_fields ),* }
            }
        }

        impl std::ops::Mul<f64> for #name {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self::Output {
                Self { #( #mul_fields ),* }
            }
        }

        impl std::ops::Div<f64> for #name {
            type Output = Self;
            fn div(self, rhs: f64) -> Self::Output {
                Self { #( #div_fields ),* }
            }
        }

        impl PartialEq for #name {
            fn eq(&self, other: &Self) -> bool {
                #( #eq_fields )&&*
            }
        }

    };

    expanded.into()
}
