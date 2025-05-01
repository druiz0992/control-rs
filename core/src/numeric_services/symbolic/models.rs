use super::error::SymbolicError;
use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

/// Evaluation Result
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum SymbolicEvalResult {
    Scalar(f64),
    Vector(DVector<f64>),
    Matrix(DMatrix<f64>),
}

impl std::fmt::Display for SymbolicEvalResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SymbolicEvalResult::Scalar(value) => write!(f, "{:?}", value),
            SymbolicEvalResult::Vector(values) => write!(f, "{:?}", values),
            SymbolicEvalResult::Matrix(values) => write!(f, "{:?}", values),
        }
    }
}

pub type SymbolicFn = Box<
    dyn Fn(Option<&HashMap<String, f64>>) -> Result<SymbolicEvalResult, SymbolicError>
        + Send
        + Sync,
>;
