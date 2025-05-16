use super::utils;
use crate::numeric_services::solver::LineSearch;
use crate::numeric_services::solver::dtos::{
    KktConditionsStatus, LagrangianMultiplier, OptimizerConfig, OptimizerParams, ProblemSpec,
    SolverResult,
};
use crate::numeric_services::symbolic::fasteval::ExprRegistry;
use crate::numeric_services::symbolic::ports::SymbolicExpr;
use crate::numeric_services::symbolic::{ExprMatrix, ExprScalar, ExprVector, TryIntoEvalResult};
use crate::physics::ModelError;
use log::info;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

const LOG_DOMAIN_RHO: &str = "log_domain_rho";
pub(crate) const LAMBDA: &str = "lagrangian_lambda_";
const MU: &str = "lagrangian_mu_";

pub struct NewtonSolverSymbolic {
    options: OptimizerConfig,
    problem: ProblemSpec,
    status: Option<KktConditionsStatus>,
    registry: Arc<ExprRegistry>,
}

impl NewtonSolverSymbolic {
    /// find x of residual_expr f(x) = 0;
    pub fn new_root_solver(
        residual_expr: &ExprVector,
        unknown_expr: &ExprVector,
        registry: &Arc<ExprRegistry>,
        options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        // jacobian = d_res / dx
        let jacobian = residual_expr.jacobian(unknown_expr)?;
        let residual_fn = residual_expr.to_fn(registry)?;
        let jacobian_fn = jacobian.to_fn(registry)?;
        let merit_fn = utils::get_merit_fn(residual_expr, registry, &options)?;

        let problem = ProblemSpec::new(residual_fn, jacobian_fn, merit_fn, 0, unknown_expr);

        Ok(Self {
            options,
            problem,
            status: None,
            registry: Arc::clone(registry),
        })
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

        // retrieve constraints
        let eq_constraints = utils::get_constraints(&eq_constraints_expr);
        let ineq_constraints = utils::get_constraints(&ineq_constraints_expr);

        // compute lagrangian L = objective + mu * eq_constraints - lambda * ineq_constraints
        let (lagrangian, mus, lambdas) =
            objective_expr.lagrangian(&eq_constraints, &ineq_constraints, MU, LAMBDA)?;
        // dL/dx = [d_objective/dx + mu * d_eq_constraints/dx - lambda * d_ineq/dx]
        let gradient = lagrangian.gradient(unknown_expr)?.wrap();

        let fn_expr = match options.get_gauss_newton() {
            false => &lagrangian,
            true => objective_expr,
        };
        // H = dL2/d2x
        let hessian = fn_expr.hessian(unknown_expr)?;
        // J = dC/dx
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

    pub fn status(&self) -> &Option<KktConditionsStatus> {
        &self.status
    }

    /// solve problem
    pub fn solve(&mut self, initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        // check if interior point minimization problem
        if self.problem.ip_residual.is_some() {
            return self.solve_ip(initial_guess);
        }
        let (residual_fn, jacobian_fn, unknown_expr) = self.problem.get_params()?;
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();
        let mut unknown_vals = utils::extend_initial_guess(initial_guess, &unknown_expr);
        let mut alpha = 1.0;

        let ls = LineSearch::new(self.options.get_line_search_opts());
        let mut status = Some(KktConditionsStatus::default());
        let registry = Arc::clone(&self.registry);
        let (n_eq, n_ineq) = (self.problem.n_eq, self.problem.n_ineq);

        for i in 0..max_iters {
            registry.insert_vars(&unknown_expr, &unknown_vals);

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
                alpha = ls.run(merit_fn, &delta, &unknown_vals, None).unwrap();
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }

            status = Some(utils::update_kkt_status(
                &fx,
                &unknown_vals,
                0.0,
                (n_eq, n_ineq),
            ));

            if self.options.get_verbose() {
                info!(
                    "iter: {}, kkt_status: {:?}, alpha: {}",
                    i, self.status, alpha
                );
            }
        }
        self.status = status;
        let (result, mus, lambdas) =
            utils::into_raw_result(&unknown_vals, initial_guess.len(), n_eq, n_ineq)?;
        Ok((
            result,
            LagrangianMultiplier::Mus(mus),
            LagrangianMultiplier::Lambdas(lambdas),
        ))
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

        // log domain interior point parameters:
        // s = sqrt(rho) * exp(sigma), lambda = sqrt(rho) * exp(-sigma)
        let rho = ExprScalar::new(LOG_DOMAIN_RHO);
        let sqrt_rho = rho.pow(0.5);

        //  (sigmas = [sigma_i], ip_C_ineq = [C_ineq(x) - s])
        let (sigmas, ip_ineq_constraints_expr) =
            utils::build_ip_params(&ineq_constraints, &sqrt_rho, registry);

        // ip_kkt_conditions = [dL/dX; C_eq; ip_C_ineq]
        let mut ip_kkt_conditions = gradient.extend(&eq_constraints);
        ip_kkt_conditions = ip_kkt_conditions.extend(&ip_ineq_constraints_expr);

        let zero_vector = ExprVector::zeros(ineq_constraints.len());
        // kkt_conditions = [dL/dx; C_eq; min(C_ineq, 0); min(lambda, 0); lambda .* C_ineq]
        let mut kkt_conditions = gradient.extend(&eq_constraints);
        kkt_conditions = kkt_conditions.extend(&ineq_constraints.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.min(&zero_vector));
        kkt_conditions = kkt_conditions.extend(&lambdas.hadamard_product(&ineq_constraints));

        // dC_eq/dx
        let ineq_jacobian = ineq_constraints.jacobian(unknown_expr)?;
        let lambda = sigmas.scalef(-1.0).exp().scale(&sqrt_rho).diagm();
        let neg_s = sigmas.exp().scale(&sqrt_rho).scalef(-1.0).diagm();

        let mut unknown_expr = unknown_expr.extend(&mus);
        unknown_expr = unknown_expr.extend(&sigmas);

        let ip_ktt_jacobian = ExprMatrix::build_ip_ktt_jacobian(
            &hessian,
            &eq_jacobian,
            &ineq_jacobian,
            lambda,
            neg_s,
            options.get_regularization_factor(),
        );

        // ip_kkt_conditions, ip_kkt_jacobian, kkt_conditions

        let residual_fn = kkt_conditions.to_fn(registry)?;
        let ip_residual_fn = ip_kkt_conditions.to_fn(registry)?;
        let ip_jacobian_fn = ip_ktt_jacobian.to_fn(registry)?;
        let ip_merit_fn = utils::get_merit_fn(&ip_kkt_conditions, registry, &options)?;

        let problem = ProblemSpec::new_ip(
            residual_fn,
            ip_residual_fn,
            ip_jacobian_fn,
            ip_merit_fn,
            eq_constraints.len(),
            ineq_constraints.len(),
            &unknown_expr,
        );

        Ok(Self {
            options,
            problem,
            status: None,
            registry: Arc::clone(registry),
        })
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
        let merit_fn = utils::get_merit_fn(&gradient, registry, &options)?;

        let problem = ProblemSpec::new(
            residual_fn,
            jacobian_fn,
            merit_fn,
            eq_constraints.len(),
            &unknown_expr,
        );

        Ok(Self {
            options,
            problem,
            status: None,
            registry: Arc::clone(registry),
        })
    }

    /// solve interior point minimization problem
    fn solve_ip(&mut self, initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        let (residual_fn, ip_residual_fn, ip_jacobian_fn, unknown_expr) =
            self.problem.get_ip_params()?;
        let max_iters = self.options.get_max_iters();
        let tolerance = self.options.get_tolerance();
        // extend unknown to include state + mus + sigmas
        let mut unknown_vals = utils::extend_initial_guess(initial_guess, &unknown_expr);
        let ls = LineSearch::new(self.options.get_line_search_opts());

        let mut rho = 0.1;
        let mut alpha = 1.0;
        let mut status = Some(KktConditionsStatus::default());
        let (n_eq, n_ineq) = (self.problem.n_eq, self.problem.n_ineq);
        let registry = Arc::clone(&self.registry);

        for n_iter in 0..max_iters {
            registry.insert_var(LOG_DOMAIN_RHO, rho);
            registry.insert_vars(&unknown_expr, &unknown_vals);

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
                alpha = ls
                    .run(ip_merit_fn, &delta, &unknown_vals, Some(ip_res.norm()))
                    .unwrap();
            }

            for (val, delta_val) in unknown_vals.iter_mut().zip(delta.iter()) {
                *val += alpha * delta_val;
            }

            let residual: DVector<f64> = residual_fn.eval(&unknown_vals).try_into_eval_result()?;
            let res_norm_inf = residual.amax();
            let ip_res: DVector<f64> = ip_residual_fn.eval(&unknown_vals).try_into_eval_result()?;
            let ip_res_norm_inf = ip_res.amax();

            status = Some(utils::update_kkt_status(
                &residual,
                &unknown_vals,
                rho,
                (n_eq, n_ineq),
            ));

            if self.options.get_verbose() {
                info!(
                    "iter: {}, kkt_status: {:?}, alpha: {}, rho: {}",
                    n_iter, self.status, alpha, rho
                );
            }

            if res_norm_inf < tolerance {
                break;
            } else if ip_res_norm_inf < tolerance {
                rho *= 0.1;
            }
        }

        self.status = status;
        let (result, mus, sigmas) =
            utils::into_raw_result(&unknown_vals, initial_guess.len(), n_eq, n_ineq)?;
        let lambdas: Vec<_> = sigmas.iter().map(|s| rho.sqrt() * (-s).exp()).collect();

        Ok((
            result,
            LagrangianMultiplier::Mus(mus),
            LagrangianMultiplier::Lambdas(lambdas),
        ))
    }
}
