use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum DerivativeType {
    Jacobian,
    Gradient,
    Hessian,
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
        Self {
            functions,
            variables,
            derivatives,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DerivativeResponse {
    pub gradient: Option<Vec<String>>,
    pub jacobian: Option<Vec<Vec<String>>>,
    pub hessian: Option<Vec<Vec<String>>>,
}
