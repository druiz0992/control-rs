pub mod linear_solver;
pub mod qp;
pub mod symbolic_solver;

pub use linear_solver::LinearSolver;
pub use qp::{QP, QPBuilder};

use crate::{numeric_services::solver::dtos::SolverResult, physics::ModelError};
pub trait Minimizer {
    fn minimize(&self, initial_guess: &[f64]) -> Result<SolverResult, ModelError>;
}

pub trait RootFinder {
    fn find_roots(&self, initial_guess: &[f64]) -> Result<SolverResult, ModelError>;
}
