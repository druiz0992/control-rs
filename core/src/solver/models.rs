use crate::{
    numeric_services::symbolic::{ExprScalar, ExprVector, SymbolicFn},
    physics::ModelError,
};

const DEFAULT_MAX_ITERS: usize = 100;
const DEFAULT_TOLERANCE: f64 = 1e-6;

#[derive(Default)]
pub struct ProblemSpec {
    pub objective: Option<SymbolicFn>,
    pub residual: Option<SymbolicFn>,
    pub jacobian: Option<SymbolicFn>,
    pub hessian: Option<SymbolicFn>,
    pub unknown_vars: Option<ExprVector>,

    pub eq_constraints: Option<SymbolicFn>,
    pub eq_jacobian: Option<SymbolicFn>,

    pub ineq_constraints: Option<SymbolicFn>,
    pub ineq_jacobian: Option<SymbolicFn>,
}

impl ProblemSpec {
    pub fn get_root_finding_params(
        &self,
    ) -> Result<(&SymbolicFn, &SymbolicFn, Vec<ExprScalar>), ModelError> {
        let residual_fn = self.residual.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Residual not configured".to_string())
        })?;
        let jacobian_fn = self.jacobian.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Jacobian not configured".to_string())
        })?;
        let unknown_vars = self.unknown_vars.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Unknown variables not configured".to_string())
        })?;

        Ok((residual_fn, jacobian_fn, unknown_vars.as_vec()))
    }
}

pub struct OptimizerConfig {
    pub max_iters: usize,
    pub tolerance: f64,
    pub penalty_factor: Option<f64>,
    pub constraint_tol: Option<f64>,
    pub alpha: Option<f64>,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iters: DEFAULT_MAX_ITERS,
            tolerance: DEFAULT_TOLERANCE,
            penalty_factor: None,
            constraint_tol: None,
            alpha: None,
        }
    }
}
