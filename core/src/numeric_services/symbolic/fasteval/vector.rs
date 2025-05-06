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

    pub fn zeros(len: usize) -> ExprVector {
        ExprVector::from_f64(vec![0.0; len])
    }

    pub fn len(&self) -> usize {
        self.vector.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vector.is_empty()
    }

    pub fn as_vec(&self) -> Vec<ExprScalar> {
        self.vector.clone()
    }

    pub fn as_str_vec(&self) -> Vec<&str> {
        self.iter().map(|s| s.as_str()).collect()
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
        if self.is_empty() {
            return Ok(ExprScalar::new(""));
        }
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

    pub fn dotf(&self, vecf: &[f64]) -> Result<ExprScalar, SymbolicError> {
        if self.len() != vecf.len() {
            return Err(SymbolicError::Other(
                "Vectors must have the same length for dot product".to_string(),
            ));
        }

        let terms: Vec<ExprScalar> = self
            .vector
            .iter()
            .zip(vecf.iter())
            .map(|(a, b)| a.scalef(*b))
            .collect();

        terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or(SymbolicError::Other("Error in dot product".to_string()))
    }

    pub fn hadamard_product(&self, other: &Self) -> Self {
        let result_vector = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.mul(b))
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn min_element(&self) -> ExprScalar {
        self.vector
            .iter()
            .cloned()
            .reduce(|a, b| a.min(&b))
            .unwrap_or_else(|| ExprScalar::from_f64(f64::INFINITY))
    }

    pub fn max_element(&self) -> ExprScalar {
        self.vector
            .iter()
            .cloned()
            .reduce(|a, b| a.max(&b))
            .unwrap_or_else(|| ExprScalar::from_f64(f64::NEG_INFINITY))
    }

    pub fn min(&self, other: &Self) -> ExprVector {
        let result_vector = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.min(b))
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn max(&self, other: &Self) -> ExprVector {
        let result_vector = self
            .vector
            .iter()
            .zip(&other.vector)
            .map(|(a, b)| a.max(b))
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn norm1(&self) -> Result<ExprScalar, SymbolicError> {
        let abs_terms: Vec<ExprScalar> = self.vector.iter().map(|v| v.abs()).collect();
        abs_terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or(SymbolicError::Other("Error calculating norm1".to_string()))
    }

    pub fn norm2(&self) -> Result<ExprScalar, SymbolicError> {
        let squared_terms: Vec<ExprScalar> = self.vector.iter().map(|v| v.mul(v)).collect();
        squared_terms
            .into_iter()
            .reduce(|a, b| a.add(&b))
            .ok_or(SymbolicError::Other("Error calculating norm2".to_string()))
            .map(|sum| sum.pow(0.5))
    }

    pub fn norm_inf(&self) -> Result<ExprScalar, SymbolicError> {
        let max_abs = self
            .vector
            .iter()
            .map(|v| v.abs())
            .reduce(|a, b| a.max(&b))
            .ok_or(SymbolicError::Other("Error calculating norm_inf".to_string()))?;
        Ok(max_abs)
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

    pub fn build_next(&self) -> Self {
        ExprVector::from_string(
            &self
                .as_vec()
                .iter()
                .map(|e| format!("next_{}", e.as_str()))
                .collect::<Vec<String>>(),
        )
    }

    pub fn vecmul_mat(&self, mat: &ExprMatrix) -> Self {
        mat.matmul_vec(self)
    }

    pub fn vecmul_matf(&self, mat: &[Vec<f64>]) -> Self {
        let result_vector = mat
            .iter()
            .map(|row| {
                row.iter()
                    .zip(&self.vector)
                    .map(|(a, b)| b.scalef(*a))
                    .reduce(|acc, item| acc.add(&item))
                    .unwrap_or_else(|| ExprScalar::from_f64(0.0))
            })
            .collect();
        Self {
            vector: result_vector,
        }
    }

    pub fn extend(&self, expr: &ExprVector) -> Self {
        let mut extended_vector = self.as_vec();
        extended_vector.extend(expr.as_vec());
        Self {
            vector: extended_vector,
        }
    }

    /// returns diagonal matrix from vector
    pub fn diagm(&self) -> ExprMatrix {
        let mut matrix = Vec::new();
        for (i, scalar) in self.vector.iter().enumerate() {
            let mut row = vec![ExprScalar::from_f64(0.0); self.len()];
            row[i] = scalar.clone();
            matrix.push(row);
        }
        ExprMatrix::from_vec(&matrix)
    }

    pub fn exp(&self) -> ExprVector {
        let result_vector = self.vector.iter().map(|v| v.exp()).collect();
        Self {
            vector: result_vector,
        }
    }
}

impl std::ops::Deref for ExprVector {
    type Target = [ExprScalar];

    fn deref(&self) -> &Self::Target {
        &self.vector
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

    fn to_fn(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError> {
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
        assert_eq!(result, ExprScalar::new("((x * x + y * y + z * z) ^ (0.5))"));
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

    #[test]
    fn test_min_element() {
        let registry = Arc::new(ExprRegistry::default());
        let expr = ExprVector::new(&["2", "4", "1", "10"]);
        let r = expr.min_element();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(1.0));
    }

    #[test]
    fn test_max_element() {
        let registry = Arc::new(ExprRegistry::default());
        let expr = ExprVector::new(&["2", "4", "1", "10"]);
        let r = expr.max_element();
        let func = r.to_fn(&registry).unwrap();
        let result = func(None).unwrap();

        assert_eq!(result, SymbolicEvalResult::Scalar(10.0));
    }

    #[test]
    fn test_min() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprVector::new(&["2", "4", "1", "10"]);
        let expr2 = ExprVector::new(&["1", "5", "1", "-10"]);
        let r = expr1.min(&expr2);
        let func = r.to_fn(&registry).unwrap();

        if let SymbolicEvalResult::Vector(result) = func(None).unwrap() {
            assert_eq!(result, DVector::from_vec(vec![1.0, 4.0, 1.0, -10.0]));
        } else {
            panic!("Expected EvalResult::Vector");
        }
    }

    #[test]
    fn test_max() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprVector::new(&["2", "4", "1", "10"]);
        let expr2 = ExprVector::new(&["1", "5", "1", "-10"]);
        let r = expr1.max(&expr2);
        let func = r.to_fn(&registry).unwrap();

        if let SymbolicEvalResult::Vector(result) = func(None).unwrap() {
            assert_eq!(result, DVector::from_vec(vec![2.0, 5.0, 1.0, 10.0]));
        } else {
            panic!("Expected EvalResult::Vector");
        }
    }

    #[test]
    fn test_hadamard_product() {
        let registry = Arc::new(ExprRegistry::default());
        let expr1 = ExprVector::new(&["2", "4", "1", "10"]);
        let expr2 = ExprVector::new(&["2", "4", "1", "10"]);
        let r = expr1.hadamard_product(&expr2);
        let func = r.to_fn(&registry).unwrap();

        if let SymbolicEvalResult::Vector(result) = func(None).unwrap() {
            assert_eq!(result, DVector::from_vec(vec![4.0, 16.0, 1.0, 100.0]));
        } else {
            panic!("Expected EvalResult::Vector");
        }
    }
}
