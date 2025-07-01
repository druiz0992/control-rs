use crate::{
    symbolic_services::symbolic::{
        ExprScalar, ExprVector, SymbolicFn, SymbolicFunction,
    },
    physics::ModelError,
};
use core::fmt;

#[derive(Default)]
pub struct ProblemSpec {
    pub residual: Option<SymbolicFunction>,
    pub ip_residual: Option<SymbolicFunction>,
    pub jacobian: Option<SymbolicFunction>,
    pub hessian: Option<SymbolicFunction>,
    pub unknown_vars: Option<ExprVector>,

    pub merit: Option<SymbolicFunction>,

    pub eq_constraints: Option<SymbolicFunction>,
    pub eq_jacobian: Option<SymbolicFunction>,
    pub n_eq: usize,

    pub ineq_constraints: Option<SymbolicFunction>,
    pub ineq_jacobian: Option<SymbolicFunction>,
    pub n_ineq: usize,
}

impl fmt::Debug for ProblemSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProblemSpec")
            .field("residual", &self.residual.is_some())
            .field("ip_residual", &self.ip_residual.is_some())
            .field("jacobian", &self.jacobian.is_some())
            .field("hessian", &self.hessian.is_some())
            .field("unknown_vars", &self.unknown_vars.as_ref().map(|v| v.len()))
            .field("merit", &self.merit.is_some())
            .field("eq_constraints", &self.eq_constraints.is_some())
            .field("n_eq_constraints", &self.n_eq)
            .field("eq_jacobian", &self.eq_jacobian.is_some())
            .field("ineq_constraints", &self.ineq_constraints.is_some())
            .field("n_ineq_constraints", &self.n_ineq)
            .field("ineq_jacobian", &self.ineq_jacobian.is_some())
            .finish()
    }
}

impl ProblemSpec {
    pub fn new(
        residual_fn: SymbolicFn,
        jacobian_fn: SymbolicFn,
        merit_fn: SymbolicFn,
        n_eq: usize,
        unknown_expr: &ExprVector,
    ) -> Self {
        ProblemSpec {
            residual: Some(SymbolicFunction::new(residual_fn, unknown_expr)),
            jacobian: Some(SymbolicFunction::new(jacobian_fn, unknown_expr)),
            merit: Some(SymbolicFunction::new(merit_fn, unknown_expr)),
            unknown_vars: Some(unknown_expr.clone()),
            n_eq,
            ..Default::default()
        }
    }
    pub fn new_ip(
        residual_fn: SymbolicFn,
        ip_residual_fn: SymbolicFn,
        ip_jacobian_fn: SymbolicFn,
        ip_merit_fn: SymbolicFn,
        n_eq: usize,
        n_ineq: usize,
        unknown_expr: &ExprVector,
    ) -> Self {
        ProblemSpec {
            residual: Some(SymbolicFunction::new(residual_fn, unknown_expr)),
            ip_residual: Some(SymbolicFunction::new(ip_residual_fn, unknown_expr)),
            jacobian: Some(SymbolicFunction::new(ip_jacobian_fn, unknown_expr)),
            merit: Some(SymbolicFunction::new(ip_merit_fn, unknown_expr)),
            unknown_vars: Some(unknown_expr.clone()),
            n_eq,
            n_ineq,
            ..Default::default()
        }
    }

    pub fn get_params(
        &self,
    ) -> Result<(&SymbolicFunction, &SymbolicFunction, Vec<ExprScalar>), ModelError> {
        let residual_fn = self.residual.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Residual not configured".to_string())
        })?;
        let jacobian_fn = self.jacobian.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Jacobian not configured".to_string())
        })?;
        let unknown_vars = self.unknown_vars.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Unknown variables not configured".to_string())
        })?;

        Ok((residual_fn, jacobian_fn, unknown_vars.to_vec()))
    }

    pub fn get_ip_params(
        &self,
    ) -> Result<
        (
            &SymbolicFunction,
            &SymbolicFunction,
            &SymbolicFunction,
            Vec<ExprScalar>,
        ),
        ModelError,
    > {
        let residual_fn = self.residual.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Residual not configured".to_string())
        })?;
        let ip_residual_fn = self.ip_residual.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration(
                "Interior Point Residual not configured".to_string(),
            )
        })?;
        let ip_jacobian_fn = self.jacobian.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration(
                "Interior Point Jacobian not configured".to_string(),
            )
        })?;
        let unknown_vars = self.unknown_vars.as_ref().ok_or_else(|| {
            ModelError::IncompleteConfiguration("Unknown variables not configured".to_string())
        })?;

        Ok((
            residual_fn,
            ip_residual_fn,
            ip_jacobian_fn,
            unknown_vars.to_vec(),
        ))
    }
}
