pub mod line_search;
pub mod models;
pub mod newton;

use crate::numeric_services::symbolic::fasteval::{ExprRegistry, registry};
use crate::physics::ModelError;
use std::sync::Arc;

/// Returns x for f(x) = 0
pub trait RootSolver<S> {
    fn solve(&self, initial_guess: &S, registry: &Arc<ExprRegistry>) -> Result<S, ModelError>;
}

pub trait Minimizer<S> {
    fn minimize(&self, initial_guess: &S, registry: &Arc<ExprRegistry>) -> Result<S, ModelError>;
}
