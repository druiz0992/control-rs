use super::{ExprMatrix, ExprScalar, ExprVector};
use crate::numeric_services::symbolic::SymbolicExpr;

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
