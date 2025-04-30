pub mod newton;
use crate::numeric_services::symbolic::{SymbolicFn, fasteval::ExprRegistry};
use std::sync::Arc;

pub trait Solver<S> {
    fn solve(
        &self,
        residual: &SymbolicFn,
        jacobian: &SymbolicFn,
        initial_guess: &S,
        registry: Arc<ExprRegistry>,
    ) -> S;
}
