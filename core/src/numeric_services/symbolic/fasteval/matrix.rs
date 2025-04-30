use super::scalar::ExprScalar;
use crate::numeric_services::symbolic::error::SymbolicError;
use crate::numeric_services::symbolic::fasteval::ExprRecord;
use crate::numeric_services::symbolic::models::{SymbolicEvalResult, SymbolicFn};
use crate::numeric_services::symbolic::ports::{SymbolicExpr, SymbolicRegistry};
use nalgebra::DMatrix;
use std::collections::HashMap;
use std::sync::Arc;

/// A struct representing a matrix of symbolic expressions (`ExprScalar`).
///
/// The `ExprMatrix` struct provides functionality for creating, manipulating,
/// and evaluating matrices of symbolic expressions. It supports operations
/// such as addition, transposition, and matrix multiplication, as well as
/// conversion to a string or a callable function.
///
/// # Trait Implementations
///
/// - `IntoIterator`:
///   Allows iteration over the rows of the matrix, either by value, reference,
///   or mutable reference.
///
/// - `SymbolicExpr`:
///   Implements symbolic expression behavior, including cloning, converting
///   to a string, and generating a callable function for evaluation.
///
/// # Examples
///
/// ```rust
/// use control_rs::numeric_services::symbolic::{SymbolicExpr, ExprMatrix};
///
/// let matrix = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
/// let transposed = matrix.transpose();
/// assert_eq!(transposed.to_string(), "[[1, 3], [2, 4]]");
///
/// let matrix1 = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
/// let matrix2 = ExprMatrix::new(&vec![&["5", "6"], &["7", "8"]]);
/// let result = matrix1.add(&matrix2);
/// assert_eq!(result.to_string(), "[[1 + 5, 2 + 6], [3 + 7, 4 + 8]]");
/// ```
///
/// # Notes
///
/// - The `to_fn` method generates a callable function that evaluates the
///   symbolic expressions in the matrix using a provided variable registry.
/// - The `matmul` method assumes that the dimensions of the matrices are
///   compatible for multiplication.

#[derive(Debug, Clone, PartialEq)]
pub struct ExprMatrix {
    matrix: Vec<Vec<ExprScalar>>,
}

impl ExprMatrix {
    pub fn new(matrix: &Vec<&[&str]>) -> Self {
        let converted_matrix = matrix
            .iter()
            .map(|row| row.iter().map(|&s| ExprScalar::new(s)).collect())
            .collect();
        ExprMatrix {
            matrix: converted_matrix,
        }
    }

    pub fn from_string(matrix: &[Vec<String>]) -> Self {
        let converted_matrix = matrix
            .iter()
            .map(|row| row.iter().map(ExprScalar::new).collect())
            .collect();
        ExprMatrix {
            matrix: converted_matrix,
        }
    }

    pub fn add(&self, other: &Self) -> Self {
        let result_matrix = self
            .matrix
            .iter()
            .zip(&other.matrix)
            .map(|(row_self, row_other)| {
                row_self
                    .iter()
                    .zip(row_other)
                    .map(|(val_self, val_other)| val_self.add(val_other))
                    .collect()
            })
            .collect();
        ExprMatrix {
            matrix: result_matrix,
        }
    }

    pub fn transpose(&self) -> Self {
        let transposed_matrix = (0..self.matrix[0].len())
            .map(|i| self.matrix.iter().map(|row| row[i].clone()).collect())
            .collect();
        ExprMatrix {
            matrix: transposed_matrix,
        }
    }

    pub fn matmul(&self, other: &Self) -> Self {
        let rows = self.matrix.len();
        let cols = other.matrix[0].len();
        let mut result_matrix = vec![vec![ExprScalar::default(); cols]; rows];

        for (i, row) in result_matrix.iter_mut().enumerate().take(rows) {
            for (j, cell) in row.iter_mut().enumerate().take(cols) {
                *cell = (0..self.matrix[0].len())
                    .map(|k| self.matrix[i][k].mul(&other.matrix[k][j]))
                    .fold(ExprScalar::default(), |acc, val| acc.add(&val));
            }
        }

        ExprMatrix {
            matrix: result_matrix,
        }
    }
}

impl IntoIterator for ExprMatrix {
    type Item = Vec<ExprScalar>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.matrix.into_iter()
    }
}

impl<'a> IntoIterator for &'a ExprMatrix {
    type Item = &'a Vec<ExprScalar>;
    type IntoIter = std::slice::Iter<'a, Vec<ExprScalar>>;

    fn into_iter(self) -> Self::IntoIter {
        self.matrix.iter()
    }
}

impl<'a> IntoIterator for &'a mut ExprMatrix {
    type Item = &'a mut Vec<ExprScalar>;
    type IntoIter = std::slice::IterMut<'a, Vec<ExprScalar>>;

    fn into_iter(self) -> Self::IntoIter {
        self.matrix.iter_mut()
    }
}

impl std::fmt::Display for ExprMatrix {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let rows = self
            .matrix
            .iter()
            .map(|row| {
                let row_str = row
                    .iter()
                    .map(|scalar| scalar.to_string())
                    .collect::<Vec<_>>()
                    .join(", ");
                format!("[{}]", row_str)
            })
            .collect::<Vec<_>>()
            .join(", ");

        write!(f, "[{}]", rows)
    }
}

impl<R> SymbolicExpr<R> for ExprMatrix
where
    R: SymbolicRegistry<Record = ExprRecord> + 'static,
{
    fn clone_box(&self) -> Box<dyn SymbolicExpr<R>> {
        Box::new(self.clone())
    }

    fn to_fn<'a>(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError> {
        let mut fn_matrix = Vec::new();

        for row in &self.matrix {
            let mut fn_row = Vec::new();
            for expr in row {
                let f = expr.to_fn(registry)?;
                fn_row.push(f);
            }
            fn_matrix.push(fn_row);
        }

        Ok(Box::new(move |vars_opt: Option<&HashMap<String, f64>>| {
            let mut values = Vec::new();
            let nrows = fn_matrix.len();
            let ncols = if nrows > 0 { fn_matrix[0].len() } else { 0 };

            for row in &fn_matrix {
                for f in row {
                    match f(vars_opt).map_err(|_| SymbolicError::EvaluationError)? {
                        SymbolicEvalResult::Scalar(val) => values.push(val),
                        _ => return Err(SymbolicError::UnexpectedResultType),
                    }
                }
            }

            Ok(SymbolicEvalResult::Matrix(DMatrix::from_row_slice(
                nrows, ncols, &values,
            )))
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::numeric_services::symbolic::fasteval::ExprRegistry;

    #[test]
    fn test_new() {
        let expr_matrix = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        assert_eq!(expr_matrix.matrix.len(), 2);
        assert_eq!(expr_matrix.matrix[0].len(), 2);
        assert_eq!(expr_matrix.matrix[0][0].to_string(), "1");
        assert_eq!(expr_matrix.matrix[1][1].to_string(), "4");
    }

    #[test]
    fn test_add() {
        let matrix1 = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        let matrix2 = ExprMatrix::new(&vec![&["5", "6"], &["7", "8"]]);
        let result = matrix1.add(&matrix2);
        assert_eq!(result.matrix[0][0].to_string(), "1 + 5");
        assert_eq!(result.matrix[0][1].to_string(), "2 + 6");
        assert_eq!(result.matrix[1][0].to_string(), "3 + 7");
        assert_eq!(result.matrix[1][1].to_string(), "4 + 8");
    }

    #[test]
    fn test_transpose() {
        let matrix = ExprMatrix::new(&vec![&["1", "2", "3"], &["4", "5", "6"]]);
        let transposed = matrix.transpose();
        assert_eq!(transposed.matrix.len(), 3);
        assert_eq!(transposed.matrix[0].len(), 2);
        assert_eq!(transposed.matrix[0][1].to_string(), "4");
        assert_eq!(transposed.matrix[2][0].to_string(), "3");
    }

    #[test]
    fn test_matmul() {
        let matrix1 = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        let matrix2 = ExprMatrix::new(&vec![&["5", "6"], &["7", "8"]]);
        let result = matrix1.matmul(&matrix2);
        assert_eq!(result.matrix[0][0].to_string(), " + 1 * 5 + 2 * 7");
        assert_eq!(result.matrix[0][1].to_string(), " + 1 * 6 + 2 * 8");
        assert_eq!(result.matrix[1][0].to_string(), " + 3 * 5 + 4 * 7");
        assert_eq!(result.matrix[1][1].to_string(), " + 3 * 6 + 4 * 8");
    }

    #[test]
    fn test_to_string() {
        let matrix = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        let matrix_str = matrix.to_string();
        assert_eq!(matrix_str, "[[1, 2], [3, 4]]");
    }

    #[test]
    fn test_to_string_complex() {
        let matrix = ExprMatrix::new(&vec![&["x", "y"], &["z", "w"]]);
        let matrix_str = matrix.to_string();
        assert_eq!(matrix_str, "[[x, y], [z, w]]");
    }

    #[test]
    fn test_to_fn() {
        let registry = Arc::new(ExprRegistry::new());
        registry.insert_var("x", 2.0);
        registry.insert_var("y", 4.0);
        registry.insert_var("z", 2.0);
        registry.insert_var("w", 4.0);
        let matrix = ExprMatrix::new(&vec![&["x+1", "y-2"], &["z+3", "w-4"]]);
        let matrix_fn = matrix.to_fn(&registry).unwrap();

        if let Ok(SymbolicEvalResult::Matrix(result)) = matrix_fn(None) {
            assert_eq!(result[(0, 0)], 3.0);
            assert_eq!(result[(0, 1)], 2.0);
            assert_eq!(result[(1, 0)], 5.0);
            assert_eq!(result[(1, 1)], 0.0);
        } else {
            panic!("Expected EvalResult::Matrix");
        }
    }
}
