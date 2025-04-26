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
    Other(String),
}

// Registry symbolic expression representation
#[derive(Debug, Clone)]
pub enum SymbolicRegistryRecord {
    Var(f64),
    Expr(Box<dyn SymbolicExpr>),
}
impl PartialEq for SymbolicRegistryRecord {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (SymbolicRegistryRecord::Var(a), SymbolicRegistryRecord::Var(b)) => a == b,
            (SymbolicRegistryRecord::Expr(a), SymbolicRegistryRecord::Expr(b)) => {
                a.to_string() == b.to_string()
            }
            _ => false,
        }
    }
}

pub trait SymbolicRegistry: Send + Sync {
    fn get(&self, expr: &str) -> Result<SymbolicRegistryRecord, SymbolicError>;
    fn insert(&mut self, name: &str, expr: SymbolicRegistryRecord);
}

pub trait SymbolicExpr: Send + Sync + std::fmt::Debug + 'static {
    fn clone_box(&self) -> Box<dyn SymbolicExpr>;
    fn to_string(&self) -> String;
    fn to_fn<'a>(
        &self,
        registry: Arc<dyn SymbolicRegistry + Send + Sync>,
    ) -> Result<
        Box<
            dyn Fn(Option<&HashMap<String, f64>>) -> Result<SymbolicEvalResult, SymbolicError>
                + Send
                + Sync,
        >,
        SymbolicError,
    >;
}

impl Clone for Box<dyn SymbolicExpr> {
    fn clone(&self) -> Box<dyn SymbolicExpr> {
        self.clone_box()
    }
}
