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
        ineq_constraints_expr: Option<ExprVector>,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        match ineq_constraints_expr {
            None => Self::new_minimization_simple(
                objective_expr,
                eq_constraints_expr,
                unknown_expr,
                registry,
                options,
            ),
            _ => Self::new_ip_minimization(
                objective_expr,
                eq_constraints_expr,
                ineq_constraints_expr,
                unknown_expr,
                registry,
                options,
            ),
        }
    }

    /// interior point minimization
    /// minimize objective_expr subject to optional eq_constraints_expr = 0 and ineq_constraints_expr >= 0
    fn new_ip_minimization(
        objective_expr: &ExprScalar,
        eq_constraints_expr: Option<ExprVector>,
        ineq_constraints_expr: Option<ExprVector>,
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
        let ineq_constraints_expr = ineq_constraints_expr
            .as_ref()
            .cloned()
            .unwrap_or(ExprVector::new(&[]));

        // compute lagrangian
        let (lagrangian, mus, lambdas) = objective_expr
            .lagrangian(&eq_constraints_expr, &ineq_constraints_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let mut sigmas = ExprVector::new(&[]);
        let rho = ExprScalar::new("log_domain_rho");

        let ip_ineq_constraints_expr = ExprVector::from_vec(
            ineq_constraints_expr
                .iter()
                .enumerate()
                .map(|(i, c)| {
                    // insert slack_sigma and slack_s in registry
                    let sigma_name = format!("slack_sigma_{}", i);
                    let sigma_expr = ExprScalar::new(&sigma_name);
                    // s = sqrt(rho) * exp(sigma);
                    let s_expr = rho.pow(0.5).mul(&sigma_expr.exp());
                    let s_name = format!("slack_s_{}", i);
                    registry.insert_scalar(&s_name, s_expr.clone());
                    let lambda_name = format!("lagrangian_lambda_{}", i);
                    let lambda_expr = rho.pow(0.5).mul(&sigma_expr.scalef(-1.0).exp());
                    registry.insert_scalar(&lambda_name, lambda_expr);
                    sigmas = sigmas.extend(&ExprVector::from_vec(vec![sigma_expr]));

                    c.sub(&s_expr).wrap()
                })
                .collect(),
        );

        // compute dL/dx
        let gradient = lagrangian
            .gradient(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?
            .wrap();

        // ip_kkt_conditions = [dL/dX; C_eq; C_ineq -s]
        let mut ip_kkt_conditions = gradient.extend(&eq_constraints_expr);
        ip_kkt_conditions = ip_kkt_conditions.extend(&ip_ineq_constraints_expr);

        let zero_vector = ExprVector::zeros(ineq_constraints_expr.len());
        let mut kkt_conditions = gradient.extend(&eq_constraints_expr);
        kkt_conditions = kkt_conditions.extend(&ineq_constraints_expr.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.hadamard_product(&ineq_constraints_expr));

        // dC_eq/dx
        let eq_jacobian = eq_constraints_expr
            .jacobian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let ineq_jacobian = ineq_constraints_expr
            .jacobian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let log_domain_scaling = sigmas.scalef(-1.0).exp().scale(&rho.pow(0.5)).diagm();

        let fn_expr = match options.get_gauss_newton() {
            false => &lagrangian,
            true => objective_expr,
        };
        let hessian = fn_expr
            .hessian(unknown_expr)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let mut unknown_expr = unknown_expr.extend(&mus);
        unknown_expr = unknown_expr.extend(&sigmas);

        let ip_ktt_jacobian = ExprMatrix::build_ip_ktt_jacobian(
            &hessian,
            &eq_jacobian,
            &ineq_jacobian,
            &log_domain_scaling,
            1e-3,
        );

        // ip_kkt_conditions, ip_kkt_jacobian, kkt_conditions

        let residual_fn = kkt_conditions
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let ip_residual_fn = ip_kkt_conditions
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let ip_jacobian_fn = ip_ktt_jacobian
            .to_fn(registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let ip_merit_fn = get_merit_fn(&ip_kkt_conditions, registry, &options)?;

        let problem = ProblemSpec::new_ip(
            residual_fn,
            ip_residual_fn,
            ip_jacobian_fn,
            ip_merit_fn,
            &unknown_expr,
        );

        Ok(Self { options, problem })
    }

    /// minimization
    /// minimize objective_expr subject to optional eq_constraints_expr = 0
    fn new_minimization_simple(
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
        let ineq_constraints_expr = ExprVector::new(&[]);

        // compute lagrangian
        let (lagrangian, mus, _) = objective_expr
            .lagrangian(&eq_constraints_expr, &ineq_constraints_expr)
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
        let unknown_expr = unknown_expr.extend(&mus);
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

    /// find x of residual_expr f(x) = 0;
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

    /// solve interior point minimization problem
    fn solve_ip(
        &self,
        initial_guess: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Vec<Vec<f64>>, ModelError> {
        let (residual_fn, ip_residual_fn, ip_jacobian_fn, unknown_expr) =
            self.problem.get_ip_params()?;
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();
        let mut history_results = Vec::with_capacity(max_iters);
        let mut unknown_vals = initial_guess.to_vec();
        if unknown_vals.len() < unknown_expr.len() {
            unknown_vals.extend(vec![0.0; unknown_expr.len() - unknown_vals.len()]);
        }
        let mut rho = 0.1;

        for _ in 0..max_iters {
            registry.insert_var("log_domain_rho", rho);
            registry.insert_vars(&unknown_expr, &unknown_vals);
            history_results.push(unknown_vals.clone());

            let ip_res = match ip_residual_fn.eval(&[]) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate internal point residual function".to_string(),
                    ));
                }
            };
            let ip_res_norm = SymbolicEvalResult::Scalar(ip_res.norm());

            let ip_jac = match ip_jacobian_fn.eval(&[]) {
                Ok(SymbolicEvalResult::Matrix(matrix)) => matrix,
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate Jacobian".to_string(),
                    ));
                }
            };

            let delta = match ip_jac.lu().solve(&(-&ip_res)) {
                Some(d) => d,
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to solve linear problem".to_string(),
                    ));
                }
            };

            let mut alpha = 1.0;
            for _ in 0..10 {
                if let Some(ip_merit_fn) = &self.problem.ip_merit {
                    let start_point: Vec<f64> = unknown_vals
                        .iter()
                        .zip(delta.iter())
                        .map(|(val, delta_val)| val + alpha * delta_val)
                        .collect();
                    let current_merit = ip_merit_fn
                        .eval(&start_point)
                        .map_err(|_| ModelError::EvaluationError)?;
                    if current_merit < ip_res_norm {
                        break;
                    }
                    alpha *= 0.5;
                }
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }

            let residual = match residual_fn.eval(&[]) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate residual function".to_string(),
                    ));
                }
            };
            let res_norm_inf = residual.amax();

            let ip_res = match ip_residual_fn.eval(&[]) {
                Ok(SymbolicEvalResult::Vector(expr)) => {
                    DVector::from_vec(expr.iter().cloned().collect())
                }
                _ => {
                    return Err(ModelError::SolverError(
                        "Failed to evaluate internal point residual function".to_string(),
                    ));
                }
            };
            let ip_res_norm_inf = ip_res.amax();

            if res_norm_inf < tolerance {
                break;
            } else if ip_res_norm_inf < tolerance {
                rho *= 0.1;
            }
        }

        Ok(history_results)
    }

    /// solve minimization problem
    pub fn solve(
        &self,
        initial_guess: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<Vec<Vec<f64>>, ModelError> {
        // check if interior point minimization problem
        if self.problem.ip_residual.is_some() {
            return self.solve_ip(initial_guess, registry);
        }
        let (residual_fn, jacobian_fn, unknown_expr) = self.problem.get_params()?;
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();
        let mut history_results = Vec::with_capacity(max_iters);
        let mut unknown_vals = initial_guess.to_vec();
        if unknown_vals.len() < unknown_expr.len() {
            unknown_vals.extend(vec![0.0; unknown_expr.len() - unknown_vals.len()]);
        }
        let mut alpha = 1.0;

        let ls = LineSearch::new(self.options.get_line_search_opts());

        for _ in 0..max_iters {
            registry.insert_vars(&unknown_expr, &unknown_vals);
            history_results.push(unknown_vals.clone());

            let fx: nalgebra::Matrix<
                f64,
                nalgebra::Dyn,
                nalgebra::Const<1>,
                nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>,
            > = match residual_fn.eval(&[]) {
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
        Ok(history_results)
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
