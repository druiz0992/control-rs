use super::line_search::LineSearch;
use super::models::{OptimizerConfig, OptimizerParams, ProblemSpec};
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::ports::SymbolicExpr;
use crate::numeric_services::symbolic::{
    ExprMatrix, ExprScalar, ExprVector, SymbolicFn, TryIntoEvalResult,
};
use crate::physics::ModelError;
use log::info;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

const LOG_DOMAIN_RHO: &str = "log_domain_rho";
const SLACK_SIGMA: &str = "slack_sigma_";
const SLACK_S: &str = "slack_s_";
const LAMBDA: &str = "lagrangian_lambda_";

pub struct NewtonSolver {
    options: OptimizerConfig,
    problem: ProblemSpec,
}

impl NewtonSolver {
    /// find x of residual_expr f(x) = 0;
    pub fn new_root_solver(
        residual_expr: &ExprVector,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        let jacobian = residual_expr.jacobian(unknown_expr)?;
        let residual_fn = residual_expr.to_fn(registry)?;
        let jacobian_fn = jacobian.to_fn(registry)?;
        let merit_fn = get_merit_fn(residual_expr, registry, &options)?;

        let problem = ProblemSpec::new(residual_fn, jacobian_fn, merit_fn, unknown_expr);

        Ok(Self { options, problem })
    }

    pub fn new_minimization(
        objective_expr: &ExprScalar,
        eq_constraints_expr: Option<ExprVector>,
        ineq_constraints_expr: Option<ExprVector>,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        // retrieve equality constraints
        let eq_constraints = get_constraints(&eq_constraints_expr);
        let ineq_constraints = get_constraints(&ineq_constraints_expr);

        // compute lagrangian
        let (lagrangian, mus, lambdas) =
            objective_expr.lagrangian(&eq_constraints, &ineq_constraints)?;
        let gradient = lagrangian.gradient(unknown_expr)?.wrap();

        let fn_expr = match options.get_gauss_newton() {
            false => &lagrangian,
            true => objective_expr,
        };
        let hessian = fn_expr.hessian(unknown_expr)?;

        let eq_jacobian = eq_constraints.jacobian(unknown_expr)?;

        let optimizer_params = OptimizerParams {
            gradient,
            eq_jacobian,
            hessian,
            eq_constraints,
            ineq_constraints,
            mus,
            lambdas,
        };

        match ineq_constraints_expr {
            None => {
                Self::new_minimization_simple(optimizer_params, unknown_expr, registry, options)
            }
            _ => Self::new_ip_minimization(optimizer_params, unknown_expr, registry, options),
        }
    }

    /// solve problem
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
        let mut unknown_vals = extend_initial_guess(initial_guess, &unknown_expr);
        let mut alpha = 1.0;

        let ls = LineSearch::new(self.options.get_line_search_opts());

        for _ in 0..max_iters {
            registry.insert_vars(&unknown_expr, &unknown_vals);
            history_results.push(unknown_vals.clone());

            let fx: DVector<f64> = residual_fn.eval(&[]).try_into_eval_result()?;
            if fx.norm() < tolerance {
                break;
            }

            let jacobian_mat: DMatrix<f64> = jacobian_fn.eval(&[]).try_into_eval_result()?;
            let delta = jacobian_mat
                .lu()
                .solve(&(-&fx))
                .ok_or(ModelError::SolverError(
                    "Failed to solve linear problem".into(),
                ))?;

            if let Some(merit_fn) = &self.problem.merit {
                alpha = ls.run(merit_fn, &delta, &unknown_vals).unwrap();
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }
        }
        history_results.push(unknown_vals.clone());
        Ok(history_results)
    }

    /// interior point minimization
    /// minimize objective_expr subject to optional eq_constraints_expr = 0 and ineq_constraints_expr >= 0
    fn new_ip_minimization(
        params: OptimizerParams,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: OptimizerConfig,
    ) -> Result<Self, ModelError> {
        let OptimizerParams {
            gradient,
            eq_jacobian,
            hessian,
            eq_constraints,
            ineq_constraints,
            mus,
            lambdas,
        } = params;
        let rho = ExprScalar::new(LOG_DOMAIN_RHO);
        let sqrt_rho = rho.pow(0.5);
        let (sigmas, ip_ineq_constraints_expr) =
            build_ip_inequality_constraints(&ineq_constraints, &sqrt_rho, registry);

        // ip_kkt_conditions = [dL/dX; C_eq; C_ineq -s]
        let mut ip_kkt_conditions = gradient.extend(&eq_constraints);
        ip_kkt_conditions = ip_kkt_conditions.extend(&ip_ineq_constraints_expr);

        let zero_vector = ExprVector::zeros(ineq_constraints.len());
        let mut kkt_conditions = gradient.extend(&eq_constraints);
        kkt_conditions = kkt_conditions.extend(&ineq_constraints.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.hadamard_product(&ineq_constraints));

        // dC_eq/dx
        let ineq_jacobian = ineq_constraints.jacobian(unknown_expr)?;
        let log_domain_scaling = sigmas.scalef(-1.0).exp().scale(&sqrt_rho).diagm();

        let mut unknown_expr = unknown_expr.extend(&mus);
        unknown_expr = unknown_expr.extend(&sigmas);

        let ip_ktt_jacobian = ExprMatrix::build_ip_ktt_jacobian(
            &hessian,
            &eq_jacobian,
            &ineq_jacobian,
            &log_domain_scaling,
            options.get_regularization_factor(),
        );

        // ip_kkt_conditions, ip_kkt_jacobian, kkt_conditions

        let residual_fn = kkt_conditions.to_fn(registry)?;
        let ip_residual_fn = ip_kkt_conditions.to_fn(registry)?;
        let ip_jacobian_fn = ip_ktt_jacobian.to_fn(registry)?;
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
        params: OptimizerParams,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: OptimizerConfig,
    ) -> Result<Self, ModelError> {
        let OptimizerParams {
            gradient,
            eq_jacobian,
            hessian,
            eq_constraints,
            mus,
            ..
        } = params;
        let residual_expr = gradient.extend(&eq_constraints);

        let unknown_expr = unknown_expr.extend(&mus);
        let ktt_jacobian = ExprMatrix::build_ktt_jacobian(
            &hessian,
            &eq_jacobian,
            options.get_regularization_factor(),
        );

        let residual_fn = residual_expr.to_fn(registry)?;
        let jacobian_fn = ktt_jacobian.to_fn(registry)?;
        let merit_fn = get_merit_fn(&gradient, registry, &options)?;

        let problem = ProblemSpec::new(residual_fn, jacobian_fn, merit_fn, &unknown_expr);

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
        let mut unknown_vals = extend_initial_guess(initial_guess, &unknown_expr);
        let mut rho = 0.1;
        let ls = LineSearch::new(self.options.get_line_search_opts());
        let mut alpha = 1.0;

        for _ in 0..max_iters {
            registry.insert_var("log_domain_rho", rho);
            registry.insert_vars(&unknown_expr, &unknown_vals);
            history_results.push(unknown_vals.clone());

            let ip_res: DVector<f64> = ip_residual_fn.eval(&[]).try_into_eval_result()?;
            let ip_jac: DMatrix<f64> = ip_jacobian_fn.eval(&[]).try_into_eval_result()?;

            let delta = ip_jac
                .clone()
                .lu()
                .solve(&(-&ip_res))
                .ok_or(ModelError::SolverError(
                    "Failed to solve linear problem".into(),
                ))?;

            if let Some(ip_merit_fn) = &self.problem.merit {
                alpha = ls.run(ip_merit_fn, &delta, &unknown_vals).unwrap();
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }

            let residual: DVector<f64> = residual_fn.eval(&[]).try_into_eval_result()?;
            let res_norm_inf = residual.amax();
            let ip_res: DVector<f64> = ip_residual_fn.eval(&[]).try_into_eval_result()?;
            let ip_res_norm_inf = ip_res.amax();

            if res_norm_inf < tolerance {
                break;
            } else if ip_res_norm_inf < tolerance {
                rho *= 0.1;
            }
        }

        history_results.push(unknown_vals.clone());
        Ok(history_results)
    }
}

fn get_merit_fn(
    residual_expr: &ExprVector,
    registry: &Arc<ExprRegistry>,
    options: &OptimizerConfig,
) -> Result<SymbolicFn, ModelError> {
    let merit_expr = options
        .get_line_search_merit()
        .unwrap_or(residual_expr.norm2()?.wrap());
    Ok(merit_expr.to_fn(registry)?)
}

fn get_constraints(expr: &Option<ExprVector>) -> ExprVector {
    expr.as_ref().cloned().unwrap_or(ExprVector::new(&[]))
}

fn create_interior_point_symbols(
    sqrt_rho: &ExprScalar,
    var_index: usize,
    registry: &Arc<ExprRegistry>,
) -> ExprScalar {
    // insert slack_sigma and slack_s in registry
    let sigma_name = format!("{}{}", SLACK_SIGMA, var_index);
    let sigma_expr = ExprScalar::new(&sigma_name);
    // s = sqrt(rho) * exp(sigma);
    let s_expr = sqrt_rho.mul(&sigma_expr.exp());
    let s_name = format!("{}{}", SLACK_S, var_index);
    registry.insert_scalar_expr(&s_name, s_expr.clone());
    let lambda_name = format!("{}{}", LAMBDA, var_index);
    let lambda_expr = sqrt_rho.mul(&sigma_expr.scalef(-1.0).exp());
    registry.insert_scalar_expr(&lambda_name, lambda_expr);
    sigma_expr
}

fn build_ip_inequality_constraints(
    ineq_constraints_expr: &ExprVector,
    sqrt_rho: &ExprScalar,
    registry: &Arc<ExprRegistry>,
) -> (ExprVector, ExprVector) {
    let mut sigmas = ExprVector::new(&[]);
    let ip_ineq_constraints_expr = ExprVector::from_vec(
        ineq_constraints_expr
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let sigma_expr = create_interior_point_symbols(sqrt_rho, i, registry);
                sigmas = sigmas.extend(&sigma_expr.as_vec());
                let s_expr = sqrt_rho.mul(&sigma_expr.exp());
                c.sub(&s_expr).wrap()
            })
            .collect(),
    );
    (sigmas, ip_ineq_constraints_expr)
}

fn extend_initial_guess(initial_guess: &[f64], unknown_expr: &[ExprScalar]) -> Vec<f64> {
    let mut unknown_vals = initial_guess.to_vec();
    if unknown_vals.len() < unknown_expr.len() {
        unknown_vals.extend(vec![0.0; unknown_expr.len() - unknown_vals.len()]);
    }
    unknown_vals
}
