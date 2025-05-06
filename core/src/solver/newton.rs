use super::RootSolver;
use super::line_search::LineSearch;
use super::models::{OptimizerConfig, ProblemSpec};
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::ports::SymbolicExpr;
use crate::numeric_services::symbolic::{
    ExprMatrix, ExprScalar, ExprVector, SymbolicEvalResult, SymbolicFn,
};
use crate::physics::ModelError;
use nalgebra::DVector;
use std::sync::Arc;

pub struct NewtonSolver {
    options: OptimizerConfig,
    problem: ProblemSpec,
}

impl NewtonSolver {
    pub fn new_minimization(
        objective_expr: &ExprScalar,
        eq_constraints_expr: Option<ExprVector>,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        // retrieve equality constraints
        let eq_constraints_expr = eq_constraints_expr
            .as_ref()
            .cloned()
            .unwrap_or(ExprVector::new(&[]));

        // compute lagrangian
        let (lagrangian, lambdas) = objective_expr
            .lagrangian(&eq_constraints_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let gradient = lagrangian
            .gradient(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?
            .wrap();

        let residual_expr = gradient.extend(&eq_constraints_expr);

        let jacobian = eq_constraints_expr
            .jacobian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let fn_expr = match options.get_gauss_newton() {
            false => &lagrangian,
            true => objective_expr,
        };
        let hessian = fn_expr
            .hessian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let unknown_expr = unknown_expr.extend(&lambdas);
        let ktt_jacobian = ExprMatrix::build_ktt_jacobian(&hessian, &jacobian, 1e-3);

        let residual_fn = residual_expr
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let jacobian_fn = ktt_jacobian
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let merit_fn = get_merit_fn(&gradient, registry, &options)?;

        let problem = ProblemSpec::new(residual_fn, jacobian_fn, merit_fn, &unknown_expr);

        Ok(Self { options, problem })
    }

    pub fn new_root_solver(
        residual_expr: &ExprVector,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        let jacobian = residual_expr
            .jacobian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let residual_fn = residual_expr
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let jacobian_fn = jacobian
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let merit_fn = get_merit_fn(residual_expr, registry, &options)?;

        let problem = ProblemSpec::new(residual_fn, jacobian_fn, merit_fn, unknown_expr);

        Ok(Self { options, problem })
    }
}

fn get_merit_fn(
    residual_expr: &ExprVector,
    registry: &Arc<ExprRegistry>,
    options: &OptimizerConfig,
) -> Result<SymbolicFn, ModelError> {
    let merit_expr = options.get_line_search_merit().unwrap_or(
        residual_expr
            .norm2()
            .map_err(|e| ModelError::Symbolic(e.to_string()))?
            .wrap(),
    );
    merit_expr
        .to_fn(registry)
        .map_err(|e| ModelError::Symbolic(e.to_string()))
}

impl RootSolver for NewtonSolver {
    fn solve(
        &self,
        initial_guess: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Vec<f64>, ModelError> {
        let (residual_fn, jacobian_fn, unknown_expr) = self.problem.get_params()?;
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();
        let mut unknown_vals = initial_guess.to_vec();
        let mut alpha = 1.0;

        let ls = LineSearch::new(self.options.get_line_search_opts());

        for _ in 0..max_iters {
            registry.insert_vars(&unknown_expr, &unknown_vals);

            let fx = match residual_fn.eval(&[]) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate residual function".to_string(),
                    ));
                }
            };
            if fx.norm() < tolerance {
                break;
            }

            let jacobian_mat = match jacobian_fn.eval(&[]) {
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

            if let Some(merit_fn) = &self.problem.merit {
                alpha = ls.run(merit_fn, &delta, &unknown_vals).unwrap();
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }
        }
        Ok(unknown_vals.to_vec())
    }
}
