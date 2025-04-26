use crate::numeric_services::symbolic::{
    SymbolicEvalResult, SymbolicExpr, SymbolicError, SymbolicRegistry, SymbolicRegistryRecord,
};
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


#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExprScalar(String);

impl ExprScalar {
    pub fn new<S: Into<String>>(s: S) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
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
    pub fn scale(&self, factor: f64) -> Self {
        Self::new(format!("{} * {}", self.0, factor))
    }
    pub fn pow(&self, exponent: f64) -> Self {
        Self::new(format!("{} ^ {}", self.0, exponent))
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
}

impl SymbolicExpr for ExprScalar {
    fn clone_box(&self) -> Box<dyn SymbolicExpr> {
        Box::new(self.clone())
    }

    fn to_string(&self) -> String {
        self.0.clone()
    }

    fn to_fn<'a>(
        &self,
        registry: Arc<dyn SymbolicRegistry + Send + Sync>,
    ) -> Result<
        Box<
            dyn Fn(Option<&HashMap<String, f64>>) -> Result<SymbolicEvalResult, SymbolicError>
                + Send
                + Sync,
        >,
        SymbolicError,
    > {
        let expr_str = self.to_string();
        let registry = Arc::clone(&registry); 

        let mut slab = Slab::new();
        let parser = Parser::new();
        let compiled = parser
            .parse(&expr_str, &mut slab.ps)
            .map_err(|_| SymbolicError::ParseError)?
            .from(&slab.ps)
            .compile(&slab.ps, &mut slab.cs); 

        Ok(Box::new(move |vars_opt: Option<&HashMap<String, f64>>| {
            compiled
                .eval(&slab, &mut |name: &str, _| {
                    // Try fast-path from vars
                    if let Some(vars) = vars_opt {
                        if let Some(val) = vars.get(name) {
                            return Some(*val);
                        }
                    }

                    // Try from registry
                    match registry.get(name) {
                        Ok(SymbolicRegistryRecord::Var(val)) => Some(val),
                        Ok(SymbolicRegistryRecord::Expr(expr)) => expr
                            .to_fn(Arc::clone(&registry))
                            .ok()
                            .and_then(|f| f(vars_opt).ok())
                            .and_then(|result| match result {
                                SymbolicEvalResult::Scalar(val) => Some(val),
                                _ => None,
                            }),
                        Err(_) => None,
                    }
                })
                .map(SymbolicEvalResult::Scalar)
                .map_err(|_| SymbolicError::EvaluationError)
        }))
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::numeric_services::symbolic::fasteval::SymbolRegistry;

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
        let result = expr.scale(2.0);
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
        let registry = Arc::new(SymbolRegistry::default());

        // Test with no variables
        let func = expr.to_fn(registry.clone_as_symbolic_registry()).unwrap();
        let result = func(None);
        assert!(result.is_err());
    }

    #[test]
    fn test_to_fn_var() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(SymbolRegistry::default());

        // Test with a variable provided
        let func = expr.to_fn(registry.clone_as_symbolic_registry()).unwrap();

        let mut vars = HashMap::new();
        vars.insert("x".to_string(), 5.0);
        let result = func(Some(&vars));
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(5.0));
    }

    #[test]
    fn test_to_fn_different_var_provided() {
        let expr = ExprScalar::new("x");
        let registry = Arc::new(SymbolRegistry::default());

        let func = expr.to_fn(registry.clone_as_symbolic_registry()).unwrap();
        let mut vars = HashMap::new();
        vars.insert("y".to_string(), 10.0);
        let result = func(Some(&vars));
        assert!(result.is_err());
    }

    #[test]
    fn test_to_fn_var_global_registry() {
        let expr = ExprScalar::new("x");
        let mut registry = SymbolRegistry::default();
        registry.insert_var("x", 3.0);
        let func = expr.to_fn(registry.clone_as_symbolic_registry()).unwrap();

        let result = func(None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(3.0));
    }

    #[test]
    fn test_to_fn_expr() {
        let mut registry = SymbolRegistry::default();

        let expr_y = ExprScalar::new("2 * x");
        registry.insert_scalar("y", expr_y);
        registry.insert_var("x", 3.0);
        let expr = ExprScalar::new("y + 1");
        let func = expr.to_fn(registry.clone_as_symbolic_registry()).unwrap();
        let result = func(None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), SymbolicEvalResult::Scalar(7.0));
    }
}
