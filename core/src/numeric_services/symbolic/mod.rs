pub mod fasteval;
use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;
use std::sync::Arc;

/// Evaluation Result
#[derive(Debug, Clone, PartialEq)]
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

/// Error representation
#[derive(Debug, Clone)]
pub enum SymbolicError {
    ParseError,
    ExprNotFound,
    EvaluationError,
    UnexpectedResultType,
    Other(String),
}

/// Definition of object that manages symbolic expression storage
pub trait SymbolicRegistry: Send + Sync + std::fmt::Debug {
    type Record;

    fn get(&self, expr: &str) -> Result<Self::Record, SymbolicError>;
    fn insert(&self, name: &str, expr: Self::Record);
}

pub type SymbolicFn = Box<
    dyn Fn(Option<&HashMap<String, f64>>) -> Result<SymbolicEvalResult, SymbolicError>
        + Send
        + Sync,
>;

pub trait SymbolicExpr: Send + Sync + std::fmt::Debug + 'static {
    type Record;

    fn clone_box(&self) -> Box<dyn SymbolicExpr<Record = Self::Record>>;
    fn to_string(&self) -> String;
    fn to_fn<'a>(
        &self,
        registry: Arc<dyn SymbolicRegistry<Record = Self::Record>>,
    ) -> Result<SymbolicFn, SymbolicError>;
}

impl<R: 'static> Clone for Box<dyn SymbolicExpr<Record = R>> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}
