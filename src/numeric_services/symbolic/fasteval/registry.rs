use crate::numeric_services::symbolic::fasteval::{ExprMatrix, ExprScalar, ExprVector};
use crate::numeric_services::symbolic::{SymbolicError, SymbolicRegistry, SymbolicRegistryRecord};
use std::collections::HashMap;
use std::sync::Arc;

/// A registry for managing symbolic variables and expressions.
///
/// The `SymbolRegistry` struct provides functionality to store and retrieve
/// symbolic variables and expressions, such as scalars, vectors, and matrices.
/// It implements the `SymbolicRegistry` trait, allowing for interaction with
/// symbolic records in a consistent manner.
///
/// # Examples
/// ```
/// use control_rs::numeric_services::symbolic::fasteval::SymbolRegistry;
/// use control_rs::numeric_services::symbolic::SymbolicRegistryRecord;
/// use control_rs::numeric_services::traits::SymbolicRegistry;
///
/// let mut registry = SymbolRegistry::new();
/// registry.insert_var("x", 42.0);
///
/// assert!(matches!(
///     registry.get("x"),
///     Ok(SymbolicRegistryRecord::Var(42.0))
/// ));
/// ```

#[derive(Debug, Clone, Default)]
pub struct SymbolRegistry {
    //  A `HashMap` that stores symbolic records, where the key is a
    //   `String` representing the name of the variable or expression, and the value
    //   is a `SymbolicRegistryRecord`.
    pub entries: HashMap<String, SymbolicRegistryRecord>,
}

impl SymbolRegistry {
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    ///   Clones the current registry and returns it as a thread-safe, dynamically
    ///   dispatched `SymbolicRegistry`.
    pub fn clone_as_symbolic_registry(&self) -> Arc<dyn SymbolicRegistry + Send + Sync> {
        Arc::new(self.clone())
    }

    ///   Inserts a variable with the given name and value into the registry.
    pub fn insert_var(&mut self, name: &str, var: f64) {
        self.insert(name, SymbolicRegistryRecord::Var(var));
    }

    ///   Inserts a scalar expression with the given name into the registry.
    pub fn insert_scalar(&mut self, name: &str, expr: ExprScalar) {
        self.insert(name, SymbolicRegistryRecord::Expr(Box::new(expr)));
    }

    ///   Inserts a vector expression with the given name into the registry.
    pub fn insert_vector(&mut self, name: &str, expr: ExprVector) {
        self.insert(name, SymbolicRegistryRecord::Expr(Box::new(expr)));
    }

    ///   Inserts a matrix expression with the given name into the registry.
    pub fn insert_matrix(&mut self, name: &str, expr: ExprMatrix) {
        self.insert(name, SymbolicRegistryRecord::Expr(Box::new(expr)));
    }
}

impl SymbolicRegistry for SymbolRegistry {
    fn get(&self, name: &str) -> Result<SymbolicRegistryRecord, SymbolicError> {
        self.entries
            .get(name)
            .cloned()
            .ok_or(SymbolicError::ExprNotFound)
    }

    fn insert(&mut self, name: &str, value: SymbolicRegistryRecord) {
        self.entries.insert(name.to_string(), value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::numeric_services::symbolic::fasteval::{ExprMatrix, ExprScalar, ExprVector};

    #[test]
    fn test_insert_var() {
        let mut registry = SymbolRegistry::new();
        registry.insert_var("x", 42.0);
        assert!(matches!(
            registry.get("x"),
            Ok(SymbolicRegistryRecord::Var(42.0))
        ));
    }

    #[test]
    fn test_insert_scalar() {
        let mut registry = SymbolRegistry::new();
        let expr = ExprScalar::new("x + 1".to_string());
        registry.insert_scalar("y", expr.clone());
        assert_eq!(
            registry.get("y").unwrap(),
            SymbolicRegistryRecord::Expr(Box::new(expr))
        );
    }

    #[test]
    fn test_insert_vector() {
        let mut registry = SymbolRegistry::new();
        let expr = ExprVector::new(&vec!["x + 1", "x+2"]);
        registry.insert_vector("y", expr.clone());
        assert_eq!(
            registry.get("y").unwrap(),
            SymbolicRegistryRecord::Expr(Box::new(expr))
        );
    }

    #[test]
    fn test_insert_matrix() {
        let mut registry = SymbolRegistry::new();
        let expr = ExprMatrix::new(&vec![&["x + 1", "x+2"], &["x+3", "x+4"]]);
        registry.insert_matrix("y", expr.clone());
        assert_eq!(
            registry.get("y").unwrap(),
            SymbolicRegistryRecord::Expr(Box::new(expr))
        );
    }

    #[test]
    fn test_get_var_existing() {
        let mut registry = SymbolRegistry::new();
        registry.insert_var("x", 42.0);
        assert!(matches!(
            registry.get("x"),
            Ok(SymbolicRegistryRecord::Var(42.0))
        ));
    }

    #[test]
    fn test_get_var_non_existing() {
        let registry = SymbolRegistry::new();
        assert!(matches!(
            registry.get("y"),
            Err(SymbolicError::ExprNotFound)
        ));
    }

    #[test]
    fn test_get_expr_non_existing() {
        let registry = SymbolRegistry::new();
        assert!(matches!(
            registry.get("z"),
            Err(SymbolicError::ExprNotFound)
        ));
    }

    #[test]
    fn test_insert_var_overwrite() {
        let mut registry = SymbolRegistry::new();
        registry.insert_var("x", 42.0);
        registry.insert_var("x", 84.0);
        assert!(matches!(
            registry.entries.get("x"),
            Some(SymbolicRegistryRecord::Var(84.0))
        ));
    }

    #[test]
    fn test_insert_expr_overwrite() {
        let mut registry = SymbolRegistry::new();
        let expr1 = ExprScalar::new("x + 1".to_string());
        let expr2 = ExprScalar::new("x + 2".to_string());
        registry.insert_scalar("y", expr1);
        registry.insert_scalar("y", expr2.clone());
        let retrieved_expr = registry.get("y").unwrap();
        assert_eq!(
            retrieved_expr,
            SymbolicRegistryRecord::Expr(Box::new(expr2))
        );
    }
}
