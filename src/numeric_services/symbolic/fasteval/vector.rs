use super::ExprScalar;
use crate::numeric_services::symbolic::SymbolicError;
use crate::numeric_services::symbolic::{SymbolicEvalResult, SymbolicExpr, SymbolicRegistry};
use nalgebra::DVector;
use std::collections::HashMap;
use std::sync::Arc;

/// A struct representing a vector of symbolic expressions (`ExprScalar`).
///
/// This struct provides methods for creating, manipulating, and evaluating
/// vectors of symbolic expressions. It supports operations such as addition,
/// subtraction, scaling, dot product, and norm calculation. It also implements
/// traits for iteration and symbolic expression evaluation.
///
/// # Examples
///
/// ```
/// use control_rs::numeric_services::symbolic::fasteval::ExprVector;
///
/// // Create a new ExprVector
/// let vector = ExprVector::new(&["x", "y", "z"]);
///
/// // Perform vector addition
/// let vec1 = ExprVector::new(&["x", "y", "z"]);
/// let vec2 = ExprVector::new(&["a", "b", "c"]);
/// let result = vec1.add(&vec2);
///
/// // Scale a vector
/// let scaled = vec1.scale(2.0);
///
/// // Calculate dot product
/// let dot_product = vec1.dot(&vec2).unwrap();
///
/// // Calculate squared norm
/// let norm2 = vec1.norm2().unwrap();
/// ```
///
/// # Trait Implementations
///
/// - Implements `IntoIterator` for consuming, borrowing, and mutably borrowing the vector.
/// - Implements `SymbolicExpr` for symbolic evaluation and conversion to a function.

#[derive(Debug, Clone, PartialEq)]
pub struct ExprVector {
    vector: Vec<ExprScalar>,
}

impl ExprVector {
    pub fn new(vector: &[&str]) -> Self {
        let expr: Vec<_> = vector.iter().map(|s| ExprScalar::new(*s)).collect();
        Self { vector: expr }
    }

    pub fn as_vec(&self) -> Vec<ExprScalar> {
        self.vector.clone()
    }

    pub fn from_vec(vec: Vec<ExprScalar>) -> Self {
        Self { vector: vec }
    }

    pub fn add(&self, other: &Self) -> Self {
        let result_vector = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.add(b))
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn sub(&self, other: &Self) -> Self {
        let result_vector = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.sub(b))
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn scale(&self, scalar: f64) -> Self {
        let result_vector = self.vector.iter().map(|v| v.scale(scalar)).collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn dot(&self, other: &Self) -> Result<ExprScalar, &'static str> {
        let terms: Vec<ExprScalar> = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.mul(b))
            .collect();

        terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or("Error in dot product")
    }

    pub fn norm2(&self) -> Result<ExprScalar, &'static str> {
        let squared_terms: Vec<ExprScalar> = self.vector.iter().map(|v| v.mul(v)).collect();
        squared_terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or("Error calculating norm")
    }
}

impl IntoIterator for ExprVector {
    type Item = ExprScalar;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.vector.into_iter()
    }
}

impl<'a> IntoIterator for &'a ExprVector {
    type Item = &'a ExprScalar;
    type IntoIter = std::slice::Iter<'a, ExprScalar>;

    fn into_iter(self) -> Self::IntoIter {
        self.vector.iter()
    }
}

impl<'a> IntoIterator for &'a mut ExprVector {
    type Item = &'a mut ExprScalar;
    type IntoIter = std::slice::IterMut<'a, ExprScalar>;

    fn into_iter(self) -> Self::IntoIter {
        self.vector.iter_mut()
    }
}

impl SymbolicExpr for ExprVector {
    fn clone_box(&self) -> Box<dyn SymbolicExpr> {
        Box::new(self.clone())
    }

    fn to_string(&self) -> String {
        format!(
            "[{}]",
            self.vector
                .iter()
                .map(|c| c.to_string())
                .collect::<Vec<_>>()
                .join(",")
        )
    }

    fn to_fn(
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
        let mut scalar_fns = Vec::new();

        for expr in &self.vector {
            let registry_clone = Arc::clone(&registry);
            let f = expr.to_fn(registry_clone)?;
            scalar_fns.push(f);
        }

        Ok(Box::new(move |vars_opt: Option<&HashMap<String, f64>>| {
            let mut values = Vec::with_capacity(scalar_fns.len());

            for f in &scalar_fns {
                match f(vars_opt)? {
                    SymbolicEvalResult::Scalar(val) => values.push(val),
                    _ => return Err(SymbolicError::EvaluationError),
                }
            }

            Ok(SymbolicEvalResult::Vector(DVector::from_vec(values)))
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::numeric_services::symbolic::fasteval::SymbolRegistry;

    #[test]
    fn test_new() {
        let vector = ExprVector::new(&["x", "y", "z"]);
        assert_eq!(
            vector.as_vec(),
            vec![
                ExprScalar::new("x"),
                ExprScalar::new("y"),
                ExprScalar::new("z")
            ]
        );
    }

    #[test]
    fn test_add() {
        let vec1 = ExprVector::new(&["x", "y", "z"]);
        let vec2 = ExprVector::new(&["a", "b", "c"]);
        let result = vec1.add(&vec2);
        assert_eq!(
            result.as_vec(),
            vec![
                ExprScalar::new("x").add(&ExprScalar::new("a")),
                ExprScalar::new("y").add(&ExprScalar::new("b")),
                ExprScalar::new("z").add(&ExprScalar::new("c"))
            ]
        );
    }

    #[test]
    fn test_sub() {
        let vec1 = ExprVector::new(&["x", "y", "z"]);
        let vec2 = ExprVector::new(&["a", "b", "c"]);
        let result = vec1.sub(&vec2);
        assert_eq!(
            result.as_vec(),
            vec![
                ExprScalar::new("x").sub(&ExprScalar::new("a")),
                ExprScalar::new("y").sub(&ExprScalar::new("b")),
                ExprScalar::new("z").sub(&ExprScalar::new("c"))
            ]
        );
    }

    #[test]
    fn test_scale() {
        let vec = ExprVector::new(&["x", "y", "z"]);
        let result = vec.scale(2.0);
        assert_eq!(
            result.as_vec(),
            vec![
                ExprScalar::new("x").scale(2.0),
                ExprScalar::new("y").scale(2.0),
                ExprScalar::new("z").scale(2.0)
            ]
        );
    }

    #[test]
    fn test_dot() {
        let vec1 = ExprVector::new(&["x", "y", "z"]);
        let vec2 = ExprVector::new(&["a", "b", "c"]);
        let result = vec1.dot(&vec2).unwrap();
        assert_eq!(
            result,
            ExprScalar::new("x")
                .mul(&ExprScalar::new("a"))
                .add(&ExprScalar::new("y").mul(&ExprScalar::new("b")))
                .add(&ExprScalar::new("z").mul(&ExprScalar::new("c")))
        );
    }

    #[test]
    fn test_norm2() {
        let vec = ExprVector::new(&["x", "y", "z"]);
        let result = vec.norm2().unwrap();
        assert_eq!(
            result,
            ExprScalar::new("x")
                .mul(&ExprScalar::new("x"))
                .add(&ExprScalar::new("y").mul(&ExprScalar::new("y")))
                .add(&ExprScalar::new("z").mul(&ExprScalar::new("z")))
        );
    }

    #[test]
    fn test_to_string() {
        let vec = ExprVector::new(&["x", "y", "z"]);
        assert_eq!(vec.to_string(), "[x,y,z]");
    }

    #[test]
    fn test_from_vec() {
        let scalars = vec![
            ExprScalar::new("x"),
            ExprScalar::new("y"),
            ExprScalar::new("z"),
        ];
        let vec = ExprVector::from_vec(scalars.clone());
        assert_eq!(vec.as_vec(), scalars);
    }

    #[test]
    fn test_into_iter() {
        let vec = ExprVector::new(&["x", "y", "z"]);
        let mut iter = vec.into_iter();
        assert_eq!(iter.next(), Some(ExprScalar::new("x")));
        assert_eq!(iter.next(), Some(ExprScalar::new("y")));
        assert_eq!(iter.next(), Some(ExprScalar::new("z")));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_iter() {
        let vec = ExprVector::new(&["x", "y", "z"]);
        let mut iter = vec.into_iter();
        assert_eq!(iter.next(), Some(ExprScalar::new("x")));
        assert_eq!(iter.next(), Some(ExprScalar::new("y")));
        assert_eq!(iter.next(), Some(ExprScalar::new("z")));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_to_string_empty() {
        let vec = ExprVector::new(&[]);
        assert_eq!(vec.to_string(), "[]");
    }

    #[test]
    fn test_to_fn() {
        let registry = SymbolRegistry::new();
        let vec_expr = ExprVector::new(&["x+2", "y+3", "z"]);

        let func = vec_expr
            .to_fn(registry.clone_as_symbolic_registry())
            .unwrap();

        let vars = HashMap::from([
            ("x".to_string(), 1.0),
            ("y".to_string(), 2.0),
            ("z".to_string(), 3.0),
        ]);

        if let SymbolicEvalResult::Vector(result) = func(Some(&vars)).unwrap() {
            assert_eq!(result, DVector::from_vec(vec![3.0, 5.0, 3.0]));
        } else {
            panic!("Expected EvalResult::Vector");
        }
    }
}
