use super::derivatives::compute_derivatives;
use crate::numeric_services::differentiation::dtos::DerivativeType;
use crate::numeric_services::symbolic::dtos::{ExprRecord, SymbolicEvalResult, SymbolicFn};
use crate::numeric_services::symbolic::error::SymbolicError;
use crate::numeric_services::symbolic::fasteval::{ExprMatrix, ExprVector};
use crate::numeric_services::symbolic::ports::{SymbolicExpr, SymbolicRegistry};
use fasteval::parser::{DEFAULT_EXPR_DEPTH_LIMIT, DEFAULT_EXPR_LEN_LIMIT};
use fasteval::{Compiler, Error, Evaler, Instruction, Parser, Slab};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

const SLAB_DEFAULT_CAPACITY: usize = 4096;
const SLAB_MAX_CAPACITY: usize = 8388608;

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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
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

    pub fn zero() -> Self {
        Self::new("0")
    }
    pub fn one() -> Self {
        Self::new("1")
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
    pub fn wrap(&self) -> Self {
        Self::new(format!("({})", self.0))
    }

    pub fn to_vec(&self) -> ExprVector {
        ExprVector::from_vec(vec![self.clone()])
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
        Self::new(format!("({}^({}))", std::f64::consts::E, self.0))
    }
    pub fn min(&self, other: &Self) -> Self {
        Self::new(format!("min({},{})", self.0, other.0))
    }
    pub fn max(&self, other: &Self) -> Self {
        Self::new(format!("max({},{})", self.0, other.0))
    }
    pub fn lt(&self, other: &Self) -> Self {
        Self::new(format!("({} < {})", self.0, other.0))
    }
    pub fn lte(&self, other: &Self) -> Self {
        Self::new(format!("({} <= {})", self.0, other.0))
    }
    pub fn gt(&self, other: &Self) -> Self {
        Self::new(format!("({} > {})", self.0, other.0))
    }
    pub fn gte(&self, other: &Self) -> Self {
        Self::new(format!("({} >= {})", self.0, other.0))
    }
    pub fn eq(&self, other: &Self) -> Self {
        Self::new(format!("({} == {})", self.0, other.0))
    }
    pub fn ne(&self, other: &Self) -> Self {
        Self::new(format!("({} != {})", self.0, other.0))
    }
    pub fn and(&self, other: &Self) -> Self {
        Self::new(format!("({} && {})", self.0, other.0))
    }
    pub fn or(&self, other: &Self) -> Self {
        Self::new(format!("({} || {})", self.0, other.0))
    }
    pub fn not(&self) -> Self {
        Self::new(format!("!({})", self.0))
    }
    pub fn select(&self, opt1: &Self, opt2: &Self) -> Self {
        Self::new(format!(
            "(({} * {}) + ({} * {}))",
            self.0,
            opt1.0,
            self.not(),
            opt2.0
        ))
    }

    pub fn sigmoid(&self, k: f64) -> Self {
        Self::new(format!(
            "1/(1+({}^({}*{})))",
            std::f64::consts::E,
            -k,
            self.0
        ))
    }
    pub fn tanh(&self, k: f64) -> Self {
        Self::new(format!("tanh({}*{})", k, self.0))
    }

    pub fn sign(&self) -> Self {
        Self::new(format!("sign({})", self.0))
    }

    pub fn smooth_sign(&self, eps: f64) -> Self {
        let eps_expr = ExprScalar::new(eps.to_string());
        let denom = self.pow(2.0).add(&eps_expr).pow(0.5);
        self.div(&denom.wrap()).wrap()
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

    pub fn lagrangian(
        &self,
        eq_constraints_expr: &ExprVector,
        ineq_constraints_expr: &ExprVector,
        mu_prefix: &str,
        lambda_prefix: &str,
    ) -> Result<(Self, ExprVector, ExprVector), SymbolicError> {
        fn make_multipliers(prefix: &str, count: usize) -> ExprVector {
            let names: Vec<String> = (0..count).map(|i| format!("{}{}", prefix, i)).collect();
            let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
            ExprVector::new(&refs)
        }

        let mus = make_multipliers(mu_prefix, eq_constraints_expr.len());
        let lambdas = make_multipliers(lambda_prefix, ineq_constraints_expr.len());

        let mut lagrangian = self.clone();

        if !eq_constraints_expr.is_empty() {
            lagrangian = lagrangian.add(&eq_constraints_expr.dot(&mus)?.wrap());
        }
        if !ineq_constraints_expr.is_empty() {
            lagrangian = lagrangian.sub(&ineq_constraints_expr.dot(&lambdas)?.wrap());
        }

        Ok((lagrangian, mus, lambdas))
    }

    pub fn compile_with_retry(&self) -> Result<(Instruction, Slab), SymbolicError> {
        let expr_str = self.as_str();
        let parser = Parser {
            expr_depth_limit: DEFAULT_EXPR_DEPTH_LIMIT * 10,
            expr_len_limit: DEFAULT_EXPR_LEN_LIMIT * 10000,
        };
        let mut capacity = SLAB_DEFAULT_CAPACITY;
        let mut slab = Slab::with_capacity(capacity);

        loop {
            if capacity > SLAB_MAX_CAPACITY {
                return Err(SymbolicError::Other("Slab reached maximum capacity".into()));
            }
            match parser.parse(expr_str, &mut slab.ps) {
                Ok(p) => {
                    let instruction = p.from(&slab.ps).compile(&slab.ps, &mut slab.cs);
                    break Ok((instruction, slab));
                }
                Err(Error::SlabOverflow) => {
                    capacity *= 2;
                    slab = Slab::with_capacity(capacity);
                    continue;
                }
                Err(e) => {
                    return Err(SymbolicError::Other(format!(
                        "Fasteval parsed failed with {}",
                        e
                    )));
                }
            };
        }
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
        let registry = Arc::clone(registry);

        let (compiled, slab) = self.compile_with_retry()?;
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
    use crate::numeric_services::symbolic::TryIntoEvalResult;
    use crate::numeric_services::symbolic::dtos::SymbolicEvalResult;
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
        assert_eq!(result.as_str(), "((x) ^ (3))");
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
        registry.insert_scalar_expr("y", expr_y);
        registry.insert_var("x", 3.0);
        let expr = ExprScalar::new("y + 1");
        let func = expr.to_fn(&registry).unwrap();
        let result = func(None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(7.0));
    }

    #[test]
    fn test_min() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("4");
        let expr3 = ExprScalar::new("1");
        let expr4 = ExprScalar::new("10");
        let r = expr1.min(&expr2).min(&expr3).min(&expr4);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_max() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("4");
        let expr3 = ExprScalar::new("1");
        let expr4 = ExprScalar::new("10");
        let r = expr1.max(&expr2).max(&expr3).max(&expr4);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(10.0));
    }

    #[test]
    fn test_lt_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("5");

        let r = expr1.lt(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_lt_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("5");

        let r = expr2.lt(&expr1);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_lte_lt() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("5");

        let r = expr1.lte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_lte_eq() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("2");

        let r = expr1.lte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }
    #[test]
    fn test_lte_gt() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("3");
        let expr2 = ExprScalar::new("2");

        let r = expr1.lte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_gt_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("2");

        let r = expr1.gt(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_gt_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("5");

        let r = expr1.gt(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_gte_lt() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("5");

        let r = expr1.gte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_gte_eq() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("2");

        let r = expr1.gte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }
    #[test]
    fn test_gte_gt() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("3");
        let expr2 = ExprScalar::new("2");

        let r = expr1.gte(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_eq_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("3");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }
    #[test]
    fn test_eq_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_ne_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }
    #[test]
    fn test_ne_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("3");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_and_true_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2).and(&expr1.gt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_and_false_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2).and(&expr1.gt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_and_false_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2).and(&expr1.lt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_and_true_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2).and(&expr1.lt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_or_true_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2).or(&expr1.gt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_or_false_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2).or(&expr1.gt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_or_false_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2).or(&expr1.lt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_or_true_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2).or(&expr1.lt(&expr2));
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_not_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.eq(&expr2).not();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_not_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");

        let r = expr1.ne(&expr2).not();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.0));
    }

    #[test]
    fn test_select_true() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");
        let r1 = ExprScalar::new("6");
        let r2 = ExprScalar::new("7");

        let r = expr1.ne(&expr2).select(&r1, &r2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(6.0));
    }

    #[test]
    fn test_select_false() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("5");
        let expr2 = ExprScalar::new("3");
        let r1 = ExprScalar::new("6");
        let r2 = ExprScalar::new("7");

        let r = expr1.eq(&expr2).select(&r1, &r2);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(7.0));
    }

    #[test]
    fn test_positive_sign() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("2");

        let r = expr1.sign();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_negative_sign() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("-2.324");

        let r = expr1.sign();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(-1.0));
    }

    #[test]
    fn test_zero_sign() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("0.0");

        let r = expr1.sign();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_sigmoid_0() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("0.0");

        let r = expr1.sigmoid(-1.0);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(0.5));
    }

    #[test]
    fn test_sigmoid_3() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprScalar::new("3.0");

        let r = expr1.sigmoid(-1.0);
        let func = r.to_fn(&registry).unwrap();
        let result = func(None);

        let eval_result: f64 = result.try_into_eval_result().unwrap();

        assert!((eval_result - 1.0 / (1.0 + (3.0_f64).exp())).abs() < 1e-6);
    }
}
