use crate::{
    numeric_services::symbolic::{SymbolicFunction, TryIntoEvalResult},
    physics::{ModelError
    },
};
use nalgebra::DMatrix;

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

pub type EvaluableDMatrix = Box<dyn Evaluable<Output = DMatrix<f64>>>;
