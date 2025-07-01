use crate::{
    symbolic_services::symbolic::{SymbolicFunction, TryIntoEvalResult},
    physics::ModelError,
};
use nalgebra::DMatrix;
use std::sync::Arc;

/// Trait for types that can be evaluated given a slice of `f64` values.
///
/// # Associated Types
/// - `Output`: The result type produced by the evaluation.
///
/// # Errors
/// Returns a `ModelError` if the evaluation fails.
pub trait Evaluable {
    type Output;

    fn evaluate(&self, vals: &[f64]) -> Result<Self::Output, ModelError>;
}

impl Evaluable for DMatrix<f64> {
    type Output = DMatrix<f64>;

    fn evaluate(&self, _vals: &[f64]) -> Result<Self::Output, ModelError> {
        Ok(self.clone())
    }
}

impl Evaluable for SymbolicFunction {
    type Output = DMatrix<f64>;

    fn evaluate(&self, vals: &[f64]) -> Result<Self::Output, ModelError> {
        Ok(self.eval(vals).try_into_eval_result()?)
    }
}

pub type NumericFunction = Arc<dyn Fn(&[f64]) -> DMatrix<f64> + Send + Sync>;
impl Evaluable for NumericFunction {
    type Output = DMatrix<f64>;

    fn evaluate(&self, vals: &[f64]) -> Result<Self::Output, ModelError> {
        Ok((self)(vals))
    }
}

pub type EvaluableMatrixFn = Box<dyn Evaluable<Output = DMatrix<f64>> + Send + Sync>;
