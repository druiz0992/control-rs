use crate::symbolic_services::symbolic::error::SymbolicError;
use crate::symbolic_services::symbolic::ports::SymbolicRegistry;
use crate::symbolic_services::symbolic::{ExprMatrix, ExprRecord, ExprScalar, ExprVector};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// A registry for managing symbolic variables and expressions.
///
/// The `ExprRegistry` struct provides functionality to store and retrieve
/// symbolic variables and expressions, such as scalars, vectors, and matrices.
/// It implements the `SymbolicRegistry` trait, allowing for interaction with
/// symbolic records in a consistent manner.
///
/// # Examples
/// ```
/// use control_rs::symbolic_services::symbolic::{ExprRecord, ExprVector, ExprRegistry, SymbolicRegistry};
///
/// let registry = ExprRegistry::new();
/// registry.insert_var("x", 42.0);
///
/// assert!(matches!(
///     registry.get("x"),
///     Ok(ExprRecord::Var(42.0))
/// ));
/// ```

#[derive(Debug, Clone, Default)]
pub struct ExprRegistry {
    //  A `HashMap` that stores symbolic records, where the key is a
    //   `String` representing the name of the variable or expression, and the value
    //   is a `ExprRecord`.
    pub entries: Arc<RwLock<HashMap<String, ExprRecord>>>,
}

impl ExprRegistry {
    pub fn new() -> Self {
        Self {
            entries: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    ///   Inserts a variable with the given name and value into the registry.
    pub fn insert_var(&self, name: &str, var: f64) {
        self.insert(name, ExprRecord::Var(var));
    }

    ///   Inserts a scalar expression with the given name into the registry.
    pub fn insert_scalar(&self, name: &str, var: f64) {
        self.insert(name, ExprRecord::Scalar(ExprScalar::new(var.to_string())));
    }
    pub fn insert_scalars(&self, name: &[&str], var: &[f64]) -> Result<(), SymbolicError> {
        if name.len() != var.len() {
            return Err(SymbolicError::Other(
                "Incorrect number of scalars during insertion.".into(),
            ));
        }

        name.iter()
            .zip(var.iter())
            .for_each(|(n, v)| self.insert_scalar(n, *v));

        Ok(())
    }

    pub fn insert_scalar_expr(&self, name: &str, expr: ExprScalar) {
        self.insert(name, ExprRecord::Scalar(expr));
    }

    ///   Inserts a vector expression with the given name into the registry.
    pub fn insert_vector(&self, name: &str, vars: &[&str]) {
        self.insert(name, ExprRecord::Vector(ExprVector::new(vars)));
    }
    pub fn insert_vector_expr(&self, name: &str, expr: ExprVector) {
        self.insert(name, ExprRecord::Vector(expr));
    }

    pub fn insert_vars(&self, expr: &[ExprScalar], vals: &[f64]) {
        for (i, name) in expr.iter().enumerate() {
            let value = if i < vals.len() { vals[i] } else { 0.0 };
            self.insert(name.as_str(), ExprRecord::Var(value));
        }
    }

    pub fn insert_vec_as_vars(&self, name: &str, vals: &[f64]) -> Result<(), SymbolicError> {
        let state_components = self.get_vector(name)?;

        for (name, value) in state_components.iter().zip(vals.iter()) {
            self.insert_var(name.as_str(), *value);
        }
        Ok(())
    }

    ///   Inserts a matrix expression with the given name into the registry.
    pub fn insert_matrix(&self, name: &str, vars: &Vec<&[&str]>) {
        let expr = ExprMatrix::new(vars);
        self.insert(name, ExprRecord::Matrix(expr));
    }
    pub fn insert_matrix_expr(&self, name: &str, expr: ExprMatrix) {
        self.insert(name, ExprRecord::Matrix(expr));
    }

    pub fn get_var(&self, name: &str) -> Result<f64, SymbolicError> {
        match self.get(name)? {
            ExprRecord::Var(var) => Ok(var),
            _ => Err(SymbolicError::ExprNotFound(name.to_string())),
        }
    }
    pub fn get_scalar(&self, name: &str) -> Result<ExprScalar, SymbolicError> {
        match self.get(name)? {
            ExprRecord::Scalar(expr) => Ok(expr),
            _ => Err(SymbolicError::ExprNotFound(name.to_string())),
        }
    }
    pub fn get_vector(&self, name: &str) -> Result<ExprVector, SymbolicError> {
        match self.get(name)? {
            ExprRecord::Vector(expr) => Ok(expr),
            _ => Err(SymbolicError::ExprNotFound(name.to_string())),
        }
    }
    pub fn get_matrix(&self, name: &str) -> Result<ExprMatrix, SymbolicError> {
        match self.get(name)? {
            ExprRecord::Matrix(expr) => Ok(expr),
            _ => Err(SymbolicError::ExprNotFound(name.to_string())),
        }
    }
}

impl SymbolicRegistry for ExprRegistry {
    type Record = ExprRecord;

    fn get(&self, name: &str) -> Result<ExprRecord, SymbolicError> {
        let entries = self.entries.read().unwrap();
        entries
            .get(name)
            .cloned()
            .ok_or(SymbolicError::ExprNotFound(name.to_string()))
    }

    fn insert(&self, name: &str, value: ExprRecord) {
        let mut entries = self.entries.write().unwrap();
        entries.insert(name.to_string(), value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::symbolic_services::symbolic::fasteval::{ExprMatrix, ExprScalar, ExprVector};

    #[test]
    fn test_insert_var() {
        let registry = ExprRegistry::new();
        registry.insert_var("x", 42.0);
        assert!(matches!(registry.get("x"), Ok(ExprRecord::Var(42.0))));
    }

    #[test]
    fn test_insert_scalar() {
        let registry = ExprRegistry::new();
        let expr = ExprScalar::new("x + 1".to_string());
        registry.insert_scalar_expr("y", expr.clone());
        assert_eq!(registry.get("y").unwrap(), ExprRecord::Scalar(expr));
    }

    #[test]
    fn test_insert_vector() {
        let registry = ExprRegistry::new();
        let expr = ExprVector::new(&["x + 1", "x+2"]);
        registry.insert_vector_expr("y", expr.clone());
        assert_eq!(registry.get("y").unwrap(), ExprRecord::Vector(expr));
    }

    #[test]
    fn test_insert_matrix() {
        let registry = ExprRegistry::new();
        let expr = ExprMatrix::new(&vec![&["x + 1", "x+2"], &["x+3", "x+4"]]);
        registry.insert_matrix_expr("y", expr.clone());
        assert_eq!(registry.get("y").unwrap(), ExprRecord::Matrix(expr));
    }

    #[test]
    fn test_get_var_existing() {
        let registry = ExprRegistry::new();
        registry.insert_var("x", 42.0);
        assert!(matches!(registry.get("x"), Ok(ExprRecord::Var(42.0))));
    }

    #[test]
    fn test_get_var_non_existing() {
        let registry = ExprRegistry::new();
        assert!(matches!(
            registry.get("y"),
            Err(SymbolicError::ExprNotFound(_))
        ));
    }

    #[test]
    fn test_get_expr_non_existing() {
        let registry = ExprRegistry::new();
        assert!(matches!(
            registry.get("z"),
            Err(SymbolicError::ExprNotFound(_))
        ));
    }

    #[test]
    fn test_insert_var_overwrite() {
        let registry = ExprRegistry::new();
        registry.insert_var("x", 42.0);
        registry.insert_var("x", 84.0);
        assert!(matches!(registry.get("x"), Ok(ExprRecord::Var(84.0))));
    }

    #[test]
    fn test_insert_expr_overwrite() {
        let registry = ExprRegistry::new();
        let expr1 = ExprScalar::new("x + 1".to_string());
        let expr2 = ExprScalar::new("x + 2".to_string());
        registry.insert_scalar_expr("y", expr1);
        registry.insert_scalar_expr("y", expr2.clone());
        let retrieved_expr = registry.get("y").unwrap();
        assert_eq!(retrieved_expr, ExprRecord::Scalar(expr2));
    }
}
