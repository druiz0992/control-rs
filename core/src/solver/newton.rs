use super::Solver;
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::{SymbolicEvalResult, SymbolicFn};
use crate::physics::traits::State;
use std::sync::Arc;

const DEFAULT_MAX_ITERS: usize = 100;
const DEFAULT_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Default)]
pub struct NewtonSolver {
    pub max_iters: usize,
    pub tolerance: f64,
}

impl NewtonSolver {
    pub fn new() -> Self {
        Self {
            max_iters: DEFAULT_MAX_ITERS,
            tolerance: DEFAULT_TOLERANCE,
        }
    }

    pub fn set_max_iters(&mut self, max_iters: usize) {
        self.max_iters = max_iters;
    }

    pub fn set_tolerance(&mut self, tolerance: f64) {
        self.tolerance = tolerance;
    }
}

impl<S> Solver<S> for NewtonSolver
where
    S: State,
{
    fn solve(
        &self,
        residual: &SymbolicFn,
        jacobian: &SymbolicFn,
        initial_guess: &S,
        registry: Arc<ExprRegistry>,
    ) -> S {
        let next_state_components = registry.get_vector("next_state").unwrap().as_vec();
        let mut state_vec = initial_guess.as_vec();
        for _ in 0..self.max_iters {
            for (name, value) in next_state_components.iter().zip(state_vec.iter()) {
                registry.insert_var(name.as_str(), *value);
            }

            let fx = match (residual)(None) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    nalgebra::DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => panic!("Failed to evaluate residual function"),
            };

            if fx.abs().sum() < self.tolerance {
                break;
            }

            let jacobian_mat = match (jacobian)(None) {
                Ok(SymbolicEvalResult::Matrix(matrix)) => matrix,
                _ => panic!("Failed to evaluate Jacobian"),
            };

            let delta = match jacobian_mat.lu().solve(&(-&fx)) {
                Some(d) => d,
                None => panic!("Failed to solve linear system"),
            };

            for (val, delta_val) in state_vec.iter_mut().zip(delta.iter()) {
                *val += delta_val;
            }
        }
        S::from_vec(state_vec)
    }
}
