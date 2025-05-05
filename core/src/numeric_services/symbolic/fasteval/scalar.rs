use super::derivatives::compute_derivatives;
use crate::numeric_services::differentiation::models::DerivativeType;
use crate::numeric_services::symbolic::error::SymbolicError;
use crate::numeric_services::symbolic::fasteval::{ExprMatrix, ExprRecord, ExprVector};
use crate::numeric_services::symbolic::models::{SymbolicEvalResult, SymbolicFn};
use crate::numeric_services::symbolic::ports::{SymbolicExpr, SymbolicRegistry};
use fasteval::parser::{DEFAULT_EXPR_DEPTH_LIMIT, DEFAULT_EXPR_LEN_LIMIT};
use fasteval::{Compiler, Evaler, Parser, Slab};
use std::collections::HashMap;
use std::sync::Arc;

/// A symbolic scalar expression represented as a string. This struct provides
/// methods to construct, manipulate, and evaluate symbolic expressions.
///
/// # Examples
///
/// ```
/// use control_rs::numeric_services::symbolic::fasteval::scalar::ExprScalar;
///
/// let expr1 = ExprScalar::new("x");
/// let expr2 = ExprScalar::new("y");
/// let result = expr1.add(&expr2);
/// assert_eq!(result.as_str(), "x + y");
/// ```
///
/// # Trait Implementations
///
/// Implements the `SymbolicExpr` trait, allowing the expression to be evaluated
/// using a symbolic registry. The `to_fn` method compiles the expression into
/// a callable function that can evaluate the expression with optional variable
/// bindings.
///
/// # Notes
///
/// This struct is designed to work with the `fasteval` crate for efficient
/// evaluation of mathematical expressions. It also integrates with a symbolic
/// registry to resolve variables and nested expressions.

#[derive(Debug, Clone, PartialEq)]
pub struct ExprScalar(String);

impl std::str::FromStr for ExprScalar {
    type Err = SymbolicError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(ExprScalar::new(s))
    }
}
impl Default for ExprScalar {
    fn default() -> Self {
        ExprScalar::new("0")
    }
}
impl ExprScalar {
    pub fn new<S: Into<String>>(s: S) -> Self {
        Self(s.into())
    }

    pub fn from_f64(value: f64) -> Self {
        Self::new(value.to_string())
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
    pub fn wrap(&self) -> Self {
        Self::new(format!("({})", self.0))
    }

    pub fn add(&self, other: &Self) -> Self {
        Self::new(format!("{} + {}", self.0, other.0))
    }
    pub fn sub(&self, other: &Self) -> Self {
        Self::new(format!("{} - {}", self.0, other.0))
    }
    pub fn mul(&self, other: &Self) -> Self {
        Self::new(format!("{} * {}", self.0, other.0))
    }
    pub fn div(&self, other: &Self) -> Self {
        Self::new(format!("{} / {}", self.0, other.0))
    }
    pub fn scale(&self, factor: &Self) -> Self {
        Self::new(format!("{} * {}", self.0, factor.0))
    }
    pub fn scalef(&self, factor: f64) -> Self {
        Self::new(format!("{} * {}", self.0, factor))
    }
    pub fn pow(&self, exponent: f64) -> Self {
        Self::new(format!("(({}) ^ ({}))", self.0, exponent))
    }
    pub fn log(&self, base: f64) -> Self {
        Self::new(format!("log_{}({})", base, self.0))
    }
    pub fn sin(&self) -> Self {
        Self::new(format!("sin({})", self.0))
    }
    pub fn cos(&self) -> Self {
        Self::new(format!("cos({})", self.0))
    }
    pub fn tan(&self) -> Self {
        Self::new(format!("tan({})", self.0))
    }
    pub fn asin(&self) -> Self {
        Self::new(format!("asin({})", self.0))
    }
    pub fn acos(&self) -> Self {
        Self::new(format!("acos({})", self.0))
    }
    pub fn atan(&self) -> Self {
        Self::new(format!("atan({})", self.0))
    }
    pub fn abs(&self) -> Self {
        Self::new(format!("abs({})", self.0))
    }
    pub fn exp(&self) -> Self {
        Self::new(format!("{}^({})", std::f64::consts::E, self.0))
    }

    pub fn gradient(&self, vars: &ExprVector) -> Result<ExprVector, SymbolicError> {
        let resp = compute_derivatives(
            &ExprRecord::Scalar(self.clone()),
            vars,
            vec![DerivativeType::Gradient],
        )
        .map_err(|e| SymbolicError::Other(e.to_string()))?;

        if let Some(gradient) = &resp.gradient {
            Ok(ExprVector::from_string(gradient))
        } else {
            Err(SymbolicError::UnexpectedResultType)
        }
    }

    pub fn hessian(&self, vars: &ExprVector) -> Result<ExprMatrix, SymbolicError> {
        let resp = compute_derivatives(
            &ExprRecord::Scalar(self.clone()),
            vars,
            vec![DerivativeType::Hessian],
        )
        .map_err(|e| SymbolicError::Other(e.to_string()))?;
        if let Some(hessian) = &resp.hessian {
            Ok(ExprMatrix::from_string(hessian))
        } else {
            Err(SymbolicError::UnexpectedResultType)
        }
    }

    pub fn new_lagrangian(
        &self,
        eq_constraints_expr: &ExprVector,
    ) -> Result<(Self, ExprVector), SymbolicError> {
        let n_constraints = eq_constraints_expr.len();
        let lambdas: Vec<String> = (0..n_constraints)
            .map(|i| format!("lambda_{}", i))
            .collect();
        let lambdas_refs: Vec<&str> = lambdas.iter().map(|s| s.as_str()).collect();
        let lambdas = ExprVector::new(&lambdas_refs);
        if n_constraints == 0 {
            return Ok((self.clone(), lambdas));
        }

        Ok((self.add(&eq_constraints_expr.dot(&lambdas)?), lambdas))
    }
}

impl std::fmt::Display for ExprScalar {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl<R> SymbolicExpr<R> for ExprScalar
where
    R: SymbolicRegistry<Record = ExprRecord> + 'static,
{
    fn clone_box(&self) -> Box<dyn SymbolicExpr<R>> {
        Box::new(self.clone())
    }

    fn to_fn(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError> {
        let expr_str = self.to_string();
        let registry = Arc::clone(registry);

        let mut slab = Slab::with_capacity(4096);
        let parser = Parser {
            expr_depth_limit: DEFAULT_EXPR_DEPTH_LIMIT,
            expr_len_limit: DEFAULT_EXPR_LEN_LIMIT * 100,
        };

        let compiled = parser
            .parse(&expr_str, &mut slab.ps)
            .map_err(|e| SymbolicError::Other(e.to_string()))?
            .from(&slab.ps)
            .compile(&slab.ps, &mut slab.cs);

        Ok(Box::new(move |vars_opt: Option<&HashMap<String, f64>>| {
            compiled
                .eval(&slab, &mut {
                    let registry = Arc::clone(&registry);

                    move |name: &str, _| {
                        // Try fast-path from vars
                        if let Some(vars) = vars_opt {
                            if let Some(val) = vars.get(name) {
                                return Some(*val);
                            }
                        }

                        match registry.get(name) {
                            Ok(ExprRecord::Var(val)) => Some(val),
                            Ok(ExprRecord::Scalar(expr)) => {
                                eval_symbolic_expr(expr, Arc::clone(&registry), vars_opt)
                            }
                            Ok(ExprRecord::Vector(expr)) => {
                                eval_symbolic_expr(expr, Arc::clone(&registry), vars_opt)
                            }
                            Ok(ExprRecord::Matrix(expr)) => {
                                eval_symbolic_expr(expr, Arc::clone(&registry), vars_opt)
                            }
                            Err(_) => None,
                        }
                    }
                })
                .map(SymbolicEvalResult::Scalar)
                .map_err(|e| SymbolicError::Other(e.to_string()))
        }))
    }
}

// Define a helper function to handle common logic
fn eval_symbolic_expr<T, R>(
    expr: T,
    registry: Arc<R>,
    vars_opt: Option<&HashMap<String, f64>>,
) -> Option<f64>
where
    T: SymbolicExpr<R>,
    R: SymbolicRegistry,
{
    expr.to_fn(&registry)
        .ok()
        .and_then(|f| f(vars_opt).ok())
        .and_then(|result| match result {
            SymbolicEvalResult::Scalar(val) => Some(val),
            _ => None,
        })
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::numeric_services::symbolic::fasteval::ExprRegistry;

    #[test]
    fn test_new() {
        let expr = ExprScalar::new("x");
        assert_eq!(expr.as_str(), "x");
    }

    #[test]
    fn test_add() {
        let expr1 = ExprScalar::new("x");
        let expr2 = ExprScalar::new("y");
        let result = expr1.add(&expr2);
        assert_eq!(result.as_str(), "x + y");
    }

    #[test]
    fn test_sub() {
        let expr1 = ExprScalar::new("x");
        let expr2 = ExprScalar::new("y");
        let result = expr1.sub(&expr2);
        assert_eq!(result.as_str(), "x - y");
    }

    #[test]
    fn test_mul() {
        let expr1 = ExprScalar::new("x");
        let expr2 = ExprScalar::new("y");
        let result = expr1.mul(&expr2);
        assert_eq!(result.as_str(), "x * y");
    }

    #[test]
    fn test_div() {
        let expr1 = ExprScalar::new("x");
        let expr2 = ExprScalar::new("y");
        let result = expr1.div(&expr2);
        assert_eq!(result.as_str(), "x / y");
    }

    #[test]
    fn test_scale() {
        let expr = ExprScalar::new("x");
        let result = expr.scalef(2.0);
        assert_eq!(result.as_str(), "x * 2");
    }

    #[test]
    fn test_pow() {
        let expr = ExprScalar::new("x");
        let result = expr.pow(3.0);
        assert_eq!(result.as_str(), "x ^ 3");
    }

    #[test]
    fn test_log() {
        let expr = ExprScalar::new("x");
        let result = expr.log(10.0);
        assert_eq!(result.as_str(), "log_10(x)");
    }

    #[test]
    fn test_trigonometric_functions() {
        let expr = ExprScalar::new("x");
        assert_eq!(expr.sin().as_str(), "sin(x)");
        assert_eq!(expr.cos().as_str(), "cos(x)");
        assert_eq!(expr.tan().as_str(), "tan(x)");
        assert_eq!(expr.asin().as_str(), "asin(x)");
        assert_eq!(expr.acos().as_str(), "acos(x)");
        assert_eq!(expr.atan().as_str(), "atan(x)");
    }

    #[test]
    fn test_to_fn_no_vars() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(ExprRegistry::default());

        // Test with no variables
        let func = expr.to_fn(&registry).unwrap();
        let result = func(None);
        assert!(result.is_err());
    }

    #[test]
    fn test_to_fn_var() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(ExprRegistry::default());

        // Test with a variable provided
        let func = expr.to_fn(&registry).unwrap();

        let mut vars = HashMap::new();
        vars.insert("x".to_string(), 5.0);
        let result = func(Some(&vars));
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(5.0));
    }

    #[test]
    fn test_to_fn_different_var_provided() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(ExprRegistry::default());

        let func = expr.to_fn(&registry).unwrap();
        let mut vars = HashMap::new();
        vars.insert("y".to_string(), 10.0);
        let result = func(Some(&vars));
        assert!(result.is_err());
    }

    #[test]
    fn test_to_fn_var_global_registry() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(ExprRegistry::default());
        registry.insert_var("x", 3.0);
        let func = expr.to_fn(&registry).unwrap();

        let result = func(None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(3.0));
    }

    #[test]
    fn test_to_fn_expr() {
        let registry = Arc::new(ExprRegistry::default());

        let expr_y = ExprScalar::new("2 * x");
        registry.insert_scalar("y", expr_y);
        registry.insert_var("x", 3.0);
        let expr = ExprScalar::new("y + 1");
        let func = expr.to_fn(&registry).unwrap();
        let result = func(None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(7.0));
    }
}
