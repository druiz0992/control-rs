use super::RootSolver;
use super::line_search::LineSearch;
use super::models::{OptimizerConfig, ProblemSpec};
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::ports::SymbolicExpr;
use crate::numeric_services::symbolic::{ExprVector, SymbolicEvalResult};
use crate::physics::ModelError;
use crate::physics::traits::State;
use std::sync::Arc;

pub struct NewtonSolver {
    options: OptimizerConfig,
    problem: ProblemSpec,
}

impl NewtonSolver {
    pub fn new_root_solver(
        residual_expr: &ExprVector,
        unknown_vars: &[&str],
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_else(OptimizerConfig::default);
        let unknown_expr = ExprVector::new(unknown_vars);

        let jacobian = residual_expr
            .jacobian(&unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let residual_fn = residual_expr
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let jacobian_fn = jacobian
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let mut problem = ProblemSpec::default();
        problem.residual = Some(residual_fn);
        problem.jacobian = Some(jacobian_fn);
        problem.unknown_vars = Some(unknown_expr);

        if let Some(merit_expr) = options.get_line_search_merit() {
            let merit_fn = merit_expr
                .to_fn(registry)
                .map_err(|e| ModelError::Symbolic(e.to_string()))?;
            problem.merit = Some(merit_fn);
        }

        Ok(Self { options, problem })
    }

    pub fn set_max_iters(&mut self, max_iters: usize) -> Result<(), ModelError> {
        self.options.set_max_iters(max_iters)
    }

    pub fn set_tolerance(&mut self, tolerance: f64) -> Result<(), ModelError> {
        self.options.set_tolerance(tolerance)
    }
}

impl<S> RootSolver<S> for NewtonSolver
where
    S: State,
{
    fn solve(&self, initial_guess: &S, registry: &Arc<ExprRegistry>) -> Result<S, ModelError> {
        let (residual_fn, jacobian_fn, unknown_expr) = self.problem.get_root_finding_params()?;
        let mut unknown_val = initial_guess.as_vec();
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();

        for _ in 0..max_iters {
            for (name, value) in unknown_expr.iter().zip(unknown_val.iter()) {
                registry.insert_var(name.as_str(), *value);
            }

            let fx = match (residual_fn)(None) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    nalgebra::DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate residual function".to_string(),
                    ));
                }
            };

            if fx.abs().sum() < tolerance {
                break;
            }

            let jacobian_mat = match (jacobian_fn)(None) {
                Ok(SymbolicEvalResult::Matrix(matrix)) => matrix,
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate Jacobian".to_string(),
                    ));
                }
            };

            let delta = match jacobian_mat.lu().solve(&(-&fx)) {
                Some(d) => d,
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to solve linear problem".to_string(),
                    ));
                }
            };

            if self.options.is_line_search_enabled() {
                let ls = LineSearch::new();
                ls.run(merit_fn, delta, registry, opts);
            }

            for (val, delta_val) in unknown_val.iter_mut().zip(delta.iter()) {
                *val += delta_val;
            }
        }
        Ok(S::from_vec(unknown_val))
    }
}
