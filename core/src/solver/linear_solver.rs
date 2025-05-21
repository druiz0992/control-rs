use nalgebra::{DMatrix, DVector};
use crate::numeric_services::solver::dtos::{LagrangianMultiplier, SolverResult};
use crate::physics::ModelError;
use crate::solver::RootFinder;

pub struct LinearSolver<'a> {
    residual: &'a Box<dyn Fn(&DVector<f64>) -> DVector<f64> + 'a>,
    jacobian: &'a DMatrix<f64>,
}

impl<'a> LinearSolver<'a> {
    pub fn new(residual: &'a Box<dyn Fn(&DVector<f64>) -> DVector<f64> + 'a>, jacobian: &'a DMatrix<f64>) -> Self {
        Self { residual, jacobian}
    }
}

impl<'a> RootFinder for LinearSolver<'a> {
    fn find_roots(&mut self, initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        let x = self
            .jacobian
            .clone()
            .lu()
            .solve(&(-(self.residual)(&DVector::zeros(initial_guess.len()))))
            .ok_or(ModelError::EvaluationError)?;
        let lm_lambda_void = LagrangianMultiplier::Lambdas(vec![]);
        let lm_mu_void = LagrangianMultiplier::Lambdas(vec![]);
        Ok((x.data.as_vec().clone(), lm_mu_void, lm_lambda_void))
    }
}
