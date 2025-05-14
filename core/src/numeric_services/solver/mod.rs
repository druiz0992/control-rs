pub mod line_search;
pub mod models;
pub mod newton;
mod newton_utils;

use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::physics::ModelError;
use std::sync::Arc;

/// Returns x for f(x) = 0
pub trait RootSolver {
    fn solve(
        &self,
        initial_guess: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Vec<f64>, ModelError>;
}

pub trait Minimizer {
    fn minimize(
        &self,
        initial_guess: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Vec<f64>, ModelError>;
}

pub use line_search::LineSearch;
pub use models::{LineSeachConfig, OptimizerConfig, OptimizerParams, ProblemSpec, KktConditionsStatus};
pub use newton::NewtonSolver;
