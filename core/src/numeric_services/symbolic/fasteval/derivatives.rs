use crate::numeric_services::differentiation::engine::DerivativeEngine;
use crate::numeric_services::differentiation::error::DerivativeError;
use crate::numeric_services::differentiation::models::{
    DerivativeRequest, DerivativeResponse, DerivativeType,
};
use crate::numeric_services::differentiation::sympy_engine::Sympy;
use crate::numeric_services::symbolic::fasteval::{ExprRecord, ExprVector};

pub fn compute_derivatives(
    expr: &ExprRecord,
    vars: &ExprVector,
    derivatives: Vec<DerivativeType>,
) -> Result<DerivativeResponse, DerivativeError> {
    let sympy = Sympy::new();
    let functions = match expr {
        ExprRecord::Var(_) | ExprRecord::Matrix(_) => Err(DerivativeError::NotFound)?,
        ExprRecord::Scalar(scalar) => {
            let functions = vec![scalar.to_string()];
            functions
        }
        ExprRecord::Vector(vec) => {
            let functions: Vec<String> = vec.as_vec().iter().map(|e| e.to_string()).collect();
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
