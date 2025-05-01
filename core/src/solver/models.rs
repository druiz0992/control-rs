use crate::{
    numeric_services::symbolic::{ExprScalar, ExprVector, SymbolicFn},
    physics::ModelError,
};

const DEFAULT_MAX_ITERS: usize = 100;
const DEFAULT_TOLERANCE: f64 = 1e-6;

const MIN_ALLOWED_MAX_ITERS: usize = 1;
const MAX_ALLOWED_MAX_ITERS: usize = 1000;

const MIN_ALLOWERD_TOLERANCE: f64 = 1e-18;

#[derive(Default)]
pub struct ProblemSpec {
    pub objective: Option<SymbolicFn>,
    pub residual: Option<SymbolicFn>,
    pub jacobian: Option<SymbolicFn>,
    pub hessian: Option<SymbolicFn>,
    pub unknown_vars: Option<ExprVector>,

    pub merit: Option<SymbolicFn>,

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

const INIT_ALPHA: f64 = 1.0;
const FACTOR_ALPHA: f64 = 0.5;
const MAX_LINESEARCH_ITERS: usize = 10;

pub struct LineSeachConfig {
    init: f64,
    factor: f64,
    max_iters: usize,
    merit_expr: Option<ExprScalar>,
}

impl Default for LineSeachConfig {
    fn default() -> Self {
        Self {
            init: INIT_ALPHA,
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
    pub fn set_init_alpha(&mut self, init_val: f64) -> Result<(), ModelError> {
        if init_val < MIN_ALLOWED_ALPHA || init_val > MAX_ALLOWED_ALPHA {
            return Err(ModelError::ConfigError(
                "Linesearch alpha out of range".to_string(),
            ));
        }
        self.init = init_val;
        Ok(())
    }

    pub fn set_factor(&mut self, factor: f64) -> Result<(), ModelError> {
        if factor < MIN_ALLOWED_FACTOR || factor > MAX_ALLOWED_FACTOR {
            return Err(ModelError::ConfigError(
                "Linesearch alpha factor out of range".to_string(),
            ));
        }
        self.factor = factor;
        Ok(())
    }

    pub fn set_max_iters(&mut self, max_iters: usize) -> Result<(), ModelError> {
        if max_iters < MIN_ALLOWED_LINESEARCH_MAX_ITERS
            || max_iters > MAX_ALLOWED_LINESEARCH_MAX_ITERS
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

    pub fn get_init_alpha(&self) -> f64 {
        self.init
    }
    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }
    pub fn get_factor(&self) -> f64 {
        self.factor
    }
}

pub struct OptimizerConfig {
    max_iters: usize,
    tolerance: f64,
    penalty_factor: Option<f64>,
    constraint_tol: Option<f64>,
    line_search: Option<LineSeachConfig>,
}

impl OptimizerConfig {
    pub fn set_max_iters(&mut self, max_iters: usize) -> Result<(), ModelError> {
        if max_iters < MIN_ALLOWED_MAX_ITERS || max_iters > MAX_ALLOWED_MAX_ITERS {
            return Err(ModelError::ConfigError(
                "Maximum number of iterations out of range".to_string(),
            ));
        }
        self.max_iters = max_iters;
        Ok(())
    }
    pub fn set_tolerance(&mut self, tolerance: f64) -> Result<(), ModelError> {
        if tolerance < MIN_ALLOWERD_TOLERANCE {
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

    pub fn set_line_search(&mut self, opts: LineSeachConfig) -> Result<(), ModelError> {
        if opts.merit_expr.is_none() {
            return Err(ModelError::ConfigError(
                "Merit expression is not configured".to_string(),
            ));
        }
        self.line_search = Some(opts);
        Ok(())
    }

    pub fn is_line_search_enabled(&self) -> bool {
        if let Some(_) = &self.line_search {
            return true;
        }
        false
    }

    pub fn get_line_search_merit(&self) -> Option<ExprScalar> {
        if let Some(line_search_opts) = &self.line_search {
            if let Some(merit_expr) = &line_search_opts.merit_expr {
                return Some(merit_expr.clone());
            }
        }
        None
    }
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iters: DEFAULT_MAX_ITERS,
            tolerance: DEFAULT_TOLERANCE,
            penalty_factor: None,
            constraint_tol: None,
            line_search: None,
        }
    }
}
