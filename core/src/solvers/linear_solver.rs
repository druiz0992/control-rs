use crate::physics::ModelError;
use crate::solvers::RootFinder;
use crate::solvers::dtos::{KktConditionsStatus, LagrangianMultiplier, SolverResult};
use nalgebra::{DMatrix, DVector};

pub type LinearResidual = Box<dyn Fn(&DVector<f64>) -> DVector<f64>>;
pub type LinearResidualRef<'a> = &'a Box<dyn Fn(&DVector<f64>) -> DVector<f64> + 'a>;
pub struct LinearSolver<'a> {
    residual: LinearResidualRef<'a>,
    jacobian: &'a DMatrix<f64>,
}

impl<'a> LinearSolver<'a> {
    pub fn new(residual: LinearResidualRef<'a>, jacobian: &'a DMatrix<f64>) -> Self {
        Self { residual, jacobian }
    }
}

impl RootFinder for LinearSolver<'_> {
    fn find_roots(&self, initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        let x = self
            .jacobian
            .clone()
            .lu()
            .solve(&(-(self.residual)(&DVector::zeros(initial_guess.len()))))
            .ok_or(ModelError::EvaluationError)?;
        let lm_lambda_void = LagrangianMultiplier::Lambdas(vec![]);
        let lm_mu_void = LagrangianMultiplier::Lambdas(vec![]);
        Ok((
            x.data.as_vec().clone(),
            KktConditionsStatus::default(),
            lm_mu_void,
            lm_lambda_void,
        ))
    }
}
