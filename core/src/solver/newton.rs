use super::RootSolver;
use super::models::{OptimizerConfig, ProblemSpec};
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::ports::SymbolicExpr;
use crate::numeric_services::symbolic::{ExprVector, SymbolicEvalResult};
use crate::physics::ModelError;
use crate::physics::traits::State;
use std::sync::Arc;

pub struct NewtonSolver {
    pub options: OptimizerConfig,
    problem: ProblemSpec,
}

impl NewtonSolver {
    pub fn new_root_solver(
        residual_expr: &ExprVector,
        unknown_vars: &[&str],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Self, ModelError> {
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

        Ok(Self {
            options: OptimizerConfig::default(),
            problem,
        })
    }

    pub fn new_minimizer(
        objective_expr: &ExprVector,
        vars: &ExprVector,
        registry: &Arc<ExprRegistry>,
    ) -> Result<Self, ModelError> {
        registry.insert_vector("vars", vars.clone());
        let jacobian = objective_expr
            .jacobian(vars)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let objective_fn = objective_expr
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let jacobian_fn = jacobian
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let mut problem = ProblemSpec::default();
        problem.objective = Some(objective_fn);
        problem.jacobian = Some(jacobian_fn);

        Ok(Self {
            options: OptimizerConfig::default(),
            problem,
        })
    }

    pub fn set_max_iters(&mut self, max_iters: usize) {
        self.options.max_iters = max_iters;
    }

    pub fn set_tolerance(&mut self, tolerance: f64) {
        self.options.tolerance = tolerance;
    }
}

impl<S> RootSolver<S> for NewtonSolver
where
    S: State,
{
    fn solve(&self, initial_guess: &S, registry: &Arc<ExprRegistry>) -> Result<S, ModelError> {
        let (residual_fn, jacobian_fn, unknown_expr) = self.problem.get_root_finding_params()?;
        let mut unknown_val = initial_guess.as_vec();

        for _ in 0..self.options.max_iters {
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

            if fx.abs().sum() < self.options.tolerance {
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

            for (val, delta_val) in unknown_val.iter_mut().zip(delta.iter()) {
                *val += delta_val;
            }
        }
        Ok(S::from_vec(unknown_val))
    }
}
