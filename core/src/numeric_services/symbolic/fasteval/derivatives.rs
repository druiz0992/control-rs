use crate::numeric_services::differentiation::DerivativeEngine;
use crate::numeric_services::differentiation::sympy_engine::Sympy;
use crate::numeric_services::differentiation::{
    DerivativeError, DerivativeRequest, DerivativeResponse, DerivativeType,
};
use crate::numeric_services::symbolic::SymbolicExpr;
use crate::numeric_services::symbolic::fasteval::{ExprRecord, ExprVector};

pub fn compute_derivatives(
    expr: &ExprRecord,
    vars: &ExprVector,
    derivatives: Vec<DerivativeType>,
) -> Result<DerivativeResponse, DerivativeError> {
    let sympy = Sympy::new();
    let functions = match expr {
        ExprRecord::Var(_) | ExprRecord::Matrix(_) => Err(DerivativeError::NotFound)?,
        ExprRecord::Vector(vec) => {
            let functions: Vec<String> = vec.as_vec().iter().map(|e| e.to_string()).collect();
            functions
        }
        ExprRecord::Scalar(scalar) => {
            let functions = vec![scalar.to_string()];
            functions
        }
    };
    let variables: Vec<String> = vars.as_vec().iter().map(|v| v.to_string()).collect();

    let req = DerivativeRequest {
        functions,
        variables,
        derivatives,
    };

    sympy.compute_derivatives(&req)
}
