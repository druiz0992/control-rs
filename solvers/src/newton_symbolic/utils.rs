use super::solver::LAMBDA;
use crate::SolverError;
use crate::dtos::{KktConditionsStatus, OptimizerConfig, RawSolverResult};
use nalgebra::DVector;
use std::sync::Arc;
use symbolic_services::symbolic::fasteval::ExprRegistry;
use symbolic_services::symbolic::ports::SymbolicExpr;
use symbolic_services::symbolic::{ExprRecord, ExprScalar, ExprVector, SymbolicFn};

const SLACK_SIGMA: &str = "slack_sigma_";
const SLACK_S: &str = "slack_s_";

pub(crate) fn update_kkt_status(
    gradient: &DVector<f64>,
    z: &[f64],
    rho: f64,
    n_constraints: (usize, usize),
) -> KktConditionsStatus {
    let (n_eq, n_ineq) = n_constraints;
    let unknown_dims = z.len() - n_eq - n_ineq;
    let sigmas = &z[unknown_dims + n_eq..];
    let lambdas: Vec<_> = sigmas.iter().map(|s| rho.sqrt() * (-s).exp()).collect();

    let lg_gradient_norm = gradient.rows(0, unknown_dims).into_owned().norm();

    let max_eq = if n_eq > 0 {
        Some(gradient.rows(unknown_dims, n_eq).into_owned().amax())
    } else {
        None
    };

    let (min_ineq, max_lambda, compl) = if n_ineq > 0 {
        let min_ineq = gradient
            .rows(unknown_dims + n_eq, n_ineq)
            .into_owned()
            .min();

        let max_lambda = gradient
            .rows(unknown_dims + n_eq + n_ineq, n_ineq)
            .into_owned()
            .amax();

        let compl = gradient
            .rows(unknown_dims + n_eq + 2 * n_ineq, n_ineq)
            .into_owned()
            .dot(&DVector::from_vec(lambdas))
            .abs();

        (Some(min_ineq), Some(max_lambda), Some(compl))
    } else {
        (None, None, None)
    };

    KktConditionsStatus {
        stationarity: lg_gradient_norm,
        max_primal_feasibility_c: max_eq,
        min_primal_feasibility_h: min_ineq,
        dual_feasibility: max_lambda,
        complementary_slackness: compl,
    }
}

pub(crate) fn get_merit_fn(
    residual_expr: &ExprVector,
    registry: &Arc<ExprRegistry>,
    options: &OptimizerConfig,
) -> Result<SymbolicFn, SolverError> {
    let merit_expr = options
        .get_line_search_merit()
        .unwrap_or(ExprRecord::Scalar(residual_expr.norm2()?.wrap()));
    if let ExprRecord::Scalar(func) = merit_expr {
        return Ok(func.to_fn(registry)?);
    }
    Err(SolverError::Unexpected("Unexpected merit function.".into()))
}

pub(crate) fn get_constraints(expr: Option<&ExprVector>) -> ExprVector {
    expr.cloned().unwrap_or(ExprVector::new(&[]))
}

/// add s = sqrt(rho) * exp(sigma), lambda = sqrt(rho) * exp(-sigma) to registry
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

/// return ([sigma_i], [ineq(x) - s])
pub(crate) fn build_ip_params(
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
                sigmas = sigmas.extend(&sigma_expr.to_vec());
                let s_expr = sqrt_rho.mul(&sigma_expr.exp());
                c.sub(&s_expr).wrap()
            })
            .collect(),
    );
    (sigmas, ip_ineq_constraints_expr)
}

pub(crate) fn extend_initial_guess(initial_guess: &[f64], unknown_expr: &[ExprScalar]) -> Vec<f64> {
    let mut unknown_vals = initial_guess.to_vec();
    if unknown_vals.len() < unknown_expr.len() {
        unknown_vals.extend(vec![0.0; unknown_expr.len() - unknown_vals.len()]);
    }
    unknown_vals
}

pub(super) fn into_raw_result(
    a: &[f64],
    x: usize,
    y: usize,
    z: usize,
) -> Result<RawSolverResult, SolverError> {
    if a.len() != x + y + z {
        return Err(SolverError::Other(
            "Total split sizes must match vector length".into(),
        ));
    }

    let (first, rest) = a.split_at(x);
    let (second, third) = rest.split_at(y);

    Ok((first.to_owned(), second.to_owned(), third.to_owned()))
}
