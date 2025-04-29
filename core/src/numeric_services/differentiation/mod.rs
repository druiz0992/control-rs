pub mod sympy_engine;
use serde::{Deserialize, Serialize};
use std::fmt;

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

/// Specification of a derivative engine service 
pub trait DerivativeEngine {
    ///   Computes derivatives for the given request. The request
    ///   specifies the functions, variables, and types of derivatives to compute (e.g., gradient,
    ///   Jacobian, Hessian). The response contains the computed derivatives or an error if the
    ///   computation fails.
    fn compute_derivatives(
        &self,
        req: &DerivativeRequest,
    ) -> Result<DerivativeResponse, DerivativeError>;
}

#[derive(Debug)]
pub enum DerivativeError {
    NotFound,
    ParseError,
    Other(String),
}

impl fmt::Display for DerivativeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DerivativeError::NotFound => write!(f, "Derivative not found"),
            DerivativeError::ParseError => write!(f, "Error parsing derivative"),
            DerivativeError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}
