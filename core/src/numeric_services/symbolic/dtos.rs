use super::{ExprMatrix, ExprScalar, ExprVector, error::SymbolicError, ports::TryIntoEvalResult};
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

impl TryIntoEvalResult<f64> for Result<SymbolicEvalResult, SymbolicError> {
    fn try_into_eval_result(self) -> Result<f64, SymbolicError> {
        match self {
            Ok(SymbolicEvalResult::Scalar(x)) => Ok(x),
            Ok(_) => Err(SymbolicError::Other("Expected scalar".into())),
            Err(e) => Err(SymbolicError::Other(e.to_string())),
        }
    }
}

impl TryIntoEvalResult<DVector<f64>> for Result<SymbolicEvalResult, SymbolicError> {
    fn try_into_eval_result(self) -> Result<DVector<f64>, SymbolicError> {
        match self {
            Ok(SymbolicEvalResult::Vector(v)) => Ok(v),
            Ok(_) => Err(SymbolicError::Other("Expected vector".into())),
            Err(e) => Err(SymbolicError::Other(e.to_string())),
        }
    }
}

impl TryIntoEvalResult<DMatrix<f64>> for Result<SymbolicEvalResult, SymbolicError> {
    fn try_into_eval_result(self) -> Result<DMatrix<f64>, SymbolicError> {
        match self {
            Ok(SymbolicEvalResult::Matrix(m)) => Ok(m),
            Ok(_) => Err(SymbolicError::Other("Expected matrix".into())),
            Err(e) => Err(SymbolicError::Other(e.to_string())),
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

impl SymbolicFunction {
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

// Registry symbolic expression representation
#[derive(Debug, Clone)]
pub enum ExprRecord {
    Var(f64),
    Scalar(ExprScalar),
    Vector(ExprVector),
    Matrix(ExprMatrix),
}

impl PartialEq for ExprRecord {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (ExprRecord::Var(a), ExprRecord::Var(b)) => a == b,
            (ExprRecord::Scalar(a), ExprRecord::Scalar(b)) => a.to_string() == b.to_string(),
            (ExprRecord::Vector(a), ExprRecord::Vector(b)) => a.to_string() == b.to_string(),
            (ExprRecord::Matrix(a), ExprRecord::Matrix(b)) => a.to_string() == b.to_string(),
            _ => false,
        }
    }
}
