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
///      - `fn as_vec(&self) -> Vec<f64>`: Converts the struct's fields into a `Vec<f64>`.
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
#[proc_macro_derive(StateOps)]
pub fn derive_state_ops(input: TokenStream) -> TokenStream {
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
    let field_strings = field_names.iter().map(|f| f.to_string());
    let num_fields = field_names.len();

    // For `new` method
    let new_fn_args = field_names.iter().map(|f| quote! { #f: f64 });
    let new_fn_init = field_names.iter().map(|f| quote! { #f });

    // For State trait
    let as_vec_fields = field_names.iter().map(|f| quote! { self.#f });
    let from_vec_fields = field_names
        .iter()
        .enumerate()
        .map(|(i, f)| quote! { #f: v[#i] });
    let label_strs = field_strings.map(|s| quote! { #s });

    // For ops
    let add_fields = field_names.iter().map(|f| quote! { #f: self.#f + rhs.#f });
    let sub_fields = field_names.iter().map(|f| quote! { #f: self.#f - rhs.#f });
    let mul_fields = field_names.iter().map(|f| quote! { #f: self.#f * rhs });
    let div_fields = field_names.iter().map(|f| quote! { #f: self.#f / rhs });
    let eq_fields = field_names.iter().map(|f| quote! { self.#f == other.#f });

    // Tuple type for `state()` method
    let tuple_type = {
        let types = std::iter::repeat(quote! { f64 })
            .take(num_fields)
            .collect::<Vec<_>>();
        quote! { ( #( #types ),* ) }
    };

    let expanded = quote! {
        impl #name {
            pub fn new( #( #new_fn_args ),* ) -> Self {
                Self { #( #new_fn_init ),* }
            }

            pub fn state(&self) -> #tuple_type {
                ( #( self.#field_names ),* )
            }
        }

        impl State for #name {
            fn as_vec(&self) -> Vec<f64> {
                vec![ #( #as_vec_fields ),* ]
            }

            fn from_vec(v: Vec<f64>) -> Self {
                assert_eq!(v.len(), #num_fields, "Expected {} elements", #num_fields);
                Self { #( #from_vec_fields ),* }
            }

            fn labels() -> &'static [&'static str] {
                &[ #( #label_strs ),* ]
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
