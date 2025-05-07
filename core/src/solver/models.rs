use crate::{
    numeric_services::symbolic::{
        ExprMatrix, ExprScalar, ExprVector, SymbolicFn, SymbolicFunction,
    },
    physics::ModelError,
};

const DEFAULT_MAX_ITERS: usize = 100;
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

    pub ineq_constraints: Option<SymbolicFunction>,
    pub ineq_jacobian: Option<SymbolicFunction>,
}

impl ProblemSpec {
    pub fn new(
        residual_fn: SymbolicFn,
        jacobian_fn: SymbolicFn,
        merit_fn: SymbolicFn,
        unknown_expr: &ExprVector,
    ) -> Self {
        ProblemSpec {
            residual: Some(SymbolicFunction::new(residual_fn, unknown_expr)),
            jacobian: Some(SymbolicFunction::new(jacobian_fn, unknown_expr)),
            merit: Some(SymbolicFunction::new(merit_fn, unknown_expr)),
            unknown_vars: Some(unknown_expr.clone()),
            ..Default::default()
        }
    }
    pub fn new_ip(
        residual_fn: SymbolicFn,
        ip_residual_fn: SymbolicFn,
        ip_jacobian_fn: SymbolicFn,
        ip_merit_fn: SymbolicFn,
        unknown_expr: &ExprVector,
    ) -> Self {
        ProblemSpec {
            residual: Some(SymbolicFunction::new(residual_fn, unknown_expr)),
            ip_residual: Some(SymbolicFunction::new(ip_residual_fn, unknown_expr)),
            jacobian: Some(SymbolicFunction::new(ip_jacobian_fn, unknown_expr)),
            merit: Some(SymbolicFunction::new(ip_merit_fn, unknown_expr)),
            unknown_vars: Some(unknown_expr.clone()),
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

        Ok((residual_fn, jacobian_fn, unknown_vars.as_vec()))
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
            unknown_vars.as_vec(),
        ))
    }
}

const FACTOR_ALPHA: f64 = 0.5;
const MAX_LINESEARCH_ITERS: usize = 10;

#[derive(Clone)]
pub struct LineSeachConfig {
    factor: f64,
    max_iters: usize,
    merit_expr: Option<ExprScalar>,
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
        let mut ls = LineSeachConfig::default();
        ls.set_factor(factor)?;
        ls.set_max_iters(max_iters)?;
        if let Some(merit_expr) = merit_expr {
            ls.set_merit(merit_expr);
        }
        Ok(ls)
    }

    pub fn set_factor(&mut self, factor: f64) -> Result<(), ModelError> {
        if !(MIN_ALLOWED_FACTOR..=MAX_ALLOWED_FACTOR).contains(&factor) {
            return Err(ModelError::ConfigError(
                "Linesearch alpha factor out of range".to_string(),
            ));
        }
        self.factor = factor;
        Ok(())
    }

    pub fn set_max_iters(&mut self, max_iters: usize) -> Result<(), ModelError> {
        if !(MIN_ALLOWED_LINESEARCH_MAX_ITERS..=MAX_ALLOWED_LINESEARCH_MAX_ITERS)
            .contains(&max_iters)
        {
            return Err(ModelError::ConfigError(
                "Linesearch max iterations out of range".to_string(),
            ));
        }
        self.max_iters = max_iters;
        Ok(())
    }

    pub fn set_merit(&mut self, merit_expr: ExprScalar) {
        self.merit_expr = Some(merit_expr);
    }

    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }
    pub fn get_factor(&self) -> f64 {
        self.factor
    }
}

#[derive(Clone)]
pub struct OptimizerConfig {
    max_iters: usize,
    tolerance: f64,
    line_search: LineSeachConfig,
    gauss_newton: bool,
}

impl OptimizerConfig {
    pub fn set_max_iters(&mut self, max_iters: usize) -> Result<(), ModelError> {
        if !(MIN_ALLOWED_MAX_ITERS..=MAX_ALLOWED_MAX_ITERS).contains(&max_iters) {
            return Err(ModelError::ConfigError(
                "Maximum number of iterations out of range".to_string(),
            ));
        }
        self.max_iters = max_iters;
        Ok(())
    }
    pub fn set_tolerance(&mut self, tolerance: f64) -> Result<(), ModelError> {
        if tolerance < MIN_ALLOWED_TOLERANCE {
            return Err(ModelError::ConfigError(
                "Tolerance out of range".to_string(),
            ));
        }
        self.tolerance = tolerance;
        Ok(())
    }

    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }

    pub fn get_tolerance(&self) -> f64 {
        self.tolerance
    }

    pub fn set_line_search_opts(&mut self, opts: LineSeachConfig) {
        self.line_search = opts;
    }

    pub fn get_line_search_merit(&self) -> Option<ExprScalar> {
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

    pub fn set_gauss_newton(&mut self, flag: bool) {
        self.gauss_newton = flag;
    }
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iters: DEFAULT_MAX_ITERS,
            tolerance: DEFAULT_TOLERANCE,
            line_search: LineSeachConfig::default(),
            gauss_newton: true,
        }
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
