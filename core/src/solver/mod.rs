pub mod qp;
pub mod symbolic_solver;

use crate::{numeric_services::solver::dtos::SolverResult, physics::ModelError};
pub trait Minimizer {
    fn minimize(&mut self, initial_guess: &[f64]) -> Result<SolverResult, ModelError>;
}

