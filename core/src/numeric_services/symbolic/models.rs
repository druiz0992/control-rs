use super::{ExprScalar, error::SymbolicError};
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

pub struct SymbolicFunction {
    func: SymbolicFn,
    param_names: Vec<String>,
}

impl<'a> SymbolicFunction {
    pub fn new(func: SymbolicFn, params: &[ExprScalar]) -> Self {
        Self {
            func,
            param_names: params.iter().map(|s| s.to_string()).collect(),
        }
    }

    pub fn eval(&self, param_vals: &[f64]) -> Result<SymbolicEvalResult, SymbolicError> {
        if param_vals.is_empty() {
            return (self.func)(None);
        }

        let registry: HashMap<String, f64> = self
            .param_names
            .iter()
            .cloned()
            .zip(param_vals.iter().copied())
            .collect();

        (self.func)(Some(&registry))
    }
}
