use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum DerivativeType {
    Jacobian,
    Gradient,
    Hessian,
}

fn custom_order(e: &DerivativeType) -> usize {
    match e {
        DerivativeType::Gradient => 0,
        DerivativeType::Jacobian => 1,
        DerivativeType::Hessian => 2,
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DerivativeRequest {
    pub variables: Vec<String>,
    pub functions: Vec<String>,
    pub derivatives: Vec<DerivativeType>,
}

impl DerivativeRequest {
    pub fn new(
        functions: Vec<String>,
        variables: Vec<String>,
        derivatives: Vec<DerivativeType>,
    ) -> Self {
        let mut filtered_derivatives = derivatives.clone();
        filtered_derivatives.sort_by_key(custom_order);
        filtered_derivatives.dedup();
        Self {
            functions,
            variables,
            derivatives: filtered_derivatives,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DerivativeResponse {
    pub gradient: Option<Vec<String>>,
    pub jacobian: Option<Vec<Vec<String>>>,
    pub hessian: Option<Vec<Vec<String>>>,
}
