use crate::{
    numeric_services::symbolic::{
        ExprMatrix, ExprRecord, ExprScalar, ExprVector, SymbolicFn, SymbolicFunction,
    },
    physics::ModelError,
};
use core::fmt;

const DEFAULT_MAX_ITERS: usize = 200;
const DEFAULT_TOLERANCE: f64 = 1e-6;

const MIN_ALLOWED_MAX_ITERS: usize = 1;
const MAX_ALLOWED_MAX_ITERS: usize = 1000;

const MIN_ALLOWED_TOLERANCE: f64 = 1e-18;

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

const FACTOR_ALPHA: f64 = 0.5;
const MAX_LINESEARCH_ITERS: usize = 10;

#[derive(Clone)]
pub struct LineSeachConfig {
    factor: f64,
    max_iters: usize,
    merit_expr: Option<ExprRecord>,
}

impl Default for LineSeachConfig {
    fn default() -> Self {
        Self {
            factor: FACTOR_ALPHA,
            max_iters: MAX_LINESEARCH_ITERS,
            merit_expr: None,
        }
    }
}

const MIN_ALLOWED_ALPHA: f64 = 1e-18;
const MAX_ALLOWED_ALPHA: f64 = 1.0;

const MIN_ALLOWED_FACTOR: f64 = MIN_ALLOWED_ALPHA;
const MAX_ALLOWED_FACTOR: f64 = MAX_ALLOWED_ALPHA;

const MIN_ALLOWED_LINESEARCH_MAX_ITERS: usize = 1;
const MAX_ALLOWED_LINESEARCH_MAX_ITERS: usize = 30;

impl LineSeachConfig {
    pub fn new(
        max_iters: usize,
        factor: f64,
        merit_expr: Option<ExprScalar>,
    ) -> Result<Self, ModelError> {
        let ls = LineSeachConfig::default()
            .set_factor(factor)?
            .set_max_iters(max_iters)?
            .set_merit(merit_expr);

        Ok(ls)
    }

    pub fn set_factor(self, factor: f64) -> Result<Self, ModelError> {
        if !(MIN_ALLOWED_FACTOR..=MAX_ALLOWED_FACTOR).contains(&factor) {
            return Err(ModelError::ConfigError(
                "Linesearch alpha factor out of range".to_string(),
            ));
        }
        let mut new = self;
        new.factor = factor;

        Ok(new)
    }

    pub fn set_max_iters(self, max_iters: usize) -> Result<Self, ModelError> {
        if !(MIN_ALLOWED_LINESEARCH_MAX_ITERS..=MAX_ALLOWED_LINESEARCH_MAX_ITERS)
            .contains(&max_iters)
        {
            return Err(ModelError::ConfigError(
                "Linesearch max iterations out of range".to_string(),
            ));
        }
        let mut new = self;
        new.max_iters = max_iters;

        Ok(new)
    }

    pub fn set_merit(mut self, merit_expr: Option<ExprScalar>) -> Self {
        self.merit_expr = merit_expr.map(ExprRecord::Scalar);
        self
    }

    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }
    pub fn get_factor(&self) -> f64 {
        self.factor
    }
}

pub struct OptimizerParams {
    pub gradient: ExprVector,
    pub eq_jacobian: ExprMatrix,
    pub hessian: ExprMatrix,
    pub eq_constraints: ExprVector,
    pub ineq_constraints: ExprVector,
    pub mus: ExprVector,
    pub lambdas: ExprVector,
}

#[derive(Clone)]
pub struct OptimizerConfig {
    max_iters: usize,
    tolerance: f64,
    line_search: LineSeachConfig,
    gauss_newton: bool,
    regularization_factor: f64,
    verbose: bool,
}

impl OptimizerConfig {
    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }

    pub fn get_tolerance(&self) -> f64 {
        self.tolerance
    }

    pub fn get_line_search_merit(&self) -> Option<ExprRecord> {
        if let Some(merit_expr) = &self.line_search.merit_expr {
            return Some(merit_expr.clone());
        }
        None
    }

    pub fn get_line_search_opts(&self) -> LineSeachConfig {
        self.line_search.clone()
    }

    pub fn get_gauss_newton(&self) -> bool {
        self.gauss_newton
    }

    pub fn get_regularization_factor(&self) -> f64 {
        self.regularization_factor
    }

    pub fn get_verbose(&self) -> bool {
        self.verbose
    }

    pub fn set_max_iters(self, max_iters: usize) -> Result<Self, ModelError> {
        if !(MIN_ALLOWED_MAX_ITERS..=MAX_ALLOWED_MAX_ITERS).contains(&max_iters) {
            return Err(ModelError::ConfigError(
                "Maximum number of iterations out of range".to_string(),
            ));
        }
        let mut new = self;
        new.max_iters = max_iters;
        Ok(new)
    }

    pub fn set_tolerance(self, tolerance: f64) -> Result<Self, ModelError> {
        if tolerance < MIN_ALLOWED_TOLERANCE {
            return Err(ModelError::ConfigError(
                "Tolerance out of range".to_string(),
            ));
        }
        let mut new = self;
        new.tolerance = tolerance;
        Ok(new)
    }

    pub fn set_line_search_opts(self, opts: LineSeachConfig) -> Self {
        let mut new = self;
        new.line_search = opts;

        new
    }

    pub fn set_gauss_newton(self, flag: bool) -> Self {
        let mut new = self;
        new.gauss_newton = flag;

        new
    }

    pub fn set_regularization_factor(self, factor: f64) -> Self {
        let mut new = self;
        new.regularization_factor = factor;

        new
    }

    pub fn set_verbose(self, flag: bool) -> Self {
        let mut new = self;
        new.verbose = flag;

        new
    }
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iters: DEFAULT_MAX_ITERS,
            tolerance: DEFAULT_TOLERANCE,
            line_search: LineSeachConfig::default(),
            gauss_newton: true,
            regularization_factor: 0.0,
            verbose: false,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct KktConditionsStatus {
    /// Lagragian gradient norm
    pub stationarity: f64,
    /// eq(x) = 0 => norm(eq(x))
    pub max_primal_feasibility_c: Option<f64>,
    /// ineq(x) => 0 => min(ineq(x))
    pub min_primal_feasibility_h: Option<f64>,
    /// lambda >= 0 => min(lambda)
    pub dual_feasibility: Option<f64>,
    /// lambda_j * ineq_j(x) = 0 => abs(dot(lambda, ineq))
    pub complementary_slackness: Option<f64>,
}

pub type SolverResult = (
    Vec<f64>,
    KktConditionsStatus,
    LagrangianMultiplier,
    LagrangianMultiplier,
);

#[derive(Debug)]
pub enum LagrangianMultiplier {
    Lambdas(Vec<f64>),
    Mus(Vec<f64>),
}
pub type RawSolverResult = (Vec<f64>, Vec<f64>, Vec<f64>);
