use super::Minimizer;
use crate::numeric_services::solver::NewtonSolverSymbolic;
use crate::numeric_services::solver::dtos::SolverResult;
use crate::physics::ModelError;

impl Minimizer for NewtonSolverSymbolic {
    fn minimize(&mut self, initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        self.solve(initial_guess)
    }
}
