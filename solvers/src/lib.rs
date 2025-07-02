pub mod dtos;
pub mod error;
pub mod linear_solver;
pub mod newton_symbolic;
pub mod osqp;
pub mod qp;

pub use error::SolverError;
pub use linear_solver::LinearSolver;
pub use newton_symbolic::solver::NewtonSolverSymbolic;
pub use osqp::{OSQPBuilder, OSQPSolver};
pub use qp::{QP, QPBuilder};

use crate::dtos::SolverResult;
pub trait Minimizer {
    fn minimize(&self, initial_guess: &[f64]) -> Result<SolverResult, SolverError>;
}

pub trait RootFinder {
    fn find_roots(&self, initial_guess: &[f64]) -> Result<SolverResult, SolverError>;
}
