use super::derivatives::compute_derivatives;
use super::{ExprMatrix, ExprScalar};
use crate::numeric_services::differentiation::models::{DerivativeResponse, DerivativeType};
use crate::numeric_services::symbolic::error::SymbolicError;
use crate::numeric_services::symbolic::fasteval::ExprRecord;
use crate::numeric_services::symbolic::models::{SymbolicEvalResult, SymbolicFn};
use crate::numeric_services::symbolic::ports::{SymbolicExpr, SymbolicRegistry};
use nalgebra::DVector;
use std::collections::HashMap;
use std::str::FromStr;
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
/// use control_rs::numeric_services::symbolic::ExprVector;
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
/// let scaled = vec1.scalef(2.0);
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
    pub fn from_string(vector: &[String]) -> Self {
        let expr: Vec<_> = vector.iter().map(ExprScalar::new).collect();
        Self { vector: expr }
    }

    pub fn as_vec(&self) -> Vec<ExprScalar> {
        self.vector.clone()
    }

    pub fn get(&self, index: usize) -> Option<ExprScalar> {
        self.as_vec().get(index).cloned()
    }

    pub fn from_vec(vec: Vec<ExprScalar>) -> Self {
        Self { vector: vec }
    }

    pub fn from_f64(vec: Vec<f64>) -> Self {
        let expr: Vec<_> = vec.into_iter().map(ExprScalar::from_f64).collect();
        Self { vector: expr }
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

    pub fn wrap(&self) -> Self {
        let result_vector = self.vector.iter().map(|v| v.wrap()).collect();
        Self {
            vector: result_vector,
        }
    }
    pub fn scale(&self, scalar: &ExprScalar) -> Self {
        let result_vector = self.vector.iter().map(|v| v.scale(scalar)).collect();
        Self {
            vector: result_vector,
        }
    }
    pub fn scalef(&self, scalar: f64) -> Self {
        let result_vector = self.vector.iter().map(|v| v.scalef(scalar)).collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn dot(&self, other: &Self) -> Result<ExprScalar, SymbolicError> {
        let terms: Vec<ExprScalar> = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.mul(b))
            .collect();

        terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or(SymbolicError::Other("Error in dot product".to_string()))
    }

    pub fn norm2(&self) -> Result<ExprScalar, SymbolicError> {
        let squared_terms: Vec<ExprScalar> = self.vector.iter().map(|v| v.mul(v)).collect();
        squared_terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or(SymbolicError::Other("Error calculating norm".to_string()))
    }

    pub fn jacobian(&self, vars: &ExprVector) -> Result<ExprMatrix, SymbolicError> {
        let resp = compute_derivatives(
            &ExprRecord::Vector(self.clone()),
            vars,
            vec![DerivativeType::Jacobian],
        )
        .map_err(|e| SymbolicError::Other(e.to_string()))?;
        if let Some(jacobian) = &resp.jacobian {
            Ok(ExprMatrix::from_string(jacobian))
        } else {
            Err(SymbolicError::UnexpectedResultType)
        }
    }
    pub fn hessian(&self, vars: &ExprVector) -> Result<ExprMatrix, SymbolicError> {
        let resp = compute_derivatives(
            &ExprRecord::Vector(self.clone()),
            vars,
            vec![DerivativeType::Jacobian],
        )
        .map_err(|e| SymbolicError::Other(e.to_string()))?;
        if let Some(hessian) = &resp.hessian {
            Ok(ExprMatrix::from_string(hessian))
        } else {
            Err(SymbolicError::UnexpectedResultType)
        }
    }

    pub fn next_state(&self) -> Self {
        ExprVector::new(
            &self
                .as_vec()
                .iter()
                .map(|e| format!("next_{}", e.as_str()))
                .collect::<Vec<String>>()
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>(),
        )
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

impl std::fmt::Display for ExprVector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let content = self
            .vector
            .iter()
            .map(|c| c.to_string())
            .collect::<Vec<_>>()
            .join(",");

        write!(f, "[{}]", content)
    }
}

impl<R> SymbolicExpr<R> for ExprVector
where
    R: SymbolicRegistry<Record = ExprRecord> + 'static,
{
    fn clone_box(&self) -> Box<dyn SymbolicExpr<R>> {
        Box::new(self.clone())
    }

    fn to_fn<'a>(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError> {
        let mut scalar_fns = Vec::new();

        for expr in &self.vector {
            let f = expr.to_fn(registry)?;
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

impl TryFrom<DerivativeResponse> for ExprVector {
    type Error = SymbolicError;

    fn try_from(resp: DerivativeResponse) -> Result<Self, Self::Error> {
        match resp.gradient {
            Some(vec) => {
                let parsed: Result<Vec<ExprScalar>, SymbolicError> =
                    vec.into_iter().map(|s| ExprScalar::from_str(&s)).collect();
                Ok(ExprVector { vector: parsed? })
            }
            None => Err(SymbolicError::ExprNotFound("gradient".to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::numeric_services::symbolic::fasteval::ExprRegistry;

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
        let result = vec.scalef(2.0);
        assert_eq!(
            result.as_vec(),
            vec![
                ExprScalar::new("x").scalef(2.0),
                ExprScalar::new("y").scalef(2.0),
                ExprScalar::new("z").scalef(2.0)
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
        let registry = Arc::new(ExprRegistry::new());
        let expr = ExprVector::new(&["x+2", "y+3", "z"]);

        let func = expr.to_fn(&registry).unwrap();

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
