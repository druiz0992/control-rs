use super::ExprVector;
use super::scalar::ExprScalar;
use super::slab::InstructionSlab;
use crate::numeric_services::symbolic::dtos::{ExprRecord, SymbolicEvalResult, SymbolicFn};
use crate::numeric_services::symbolic::error::SymbolicError;
use crate::numeric_services::symbolic::ports::{SymbolicExpr, SymbolicRegistry};
use crate::numeric_services::symbolic::{ExprRegistry, SymbolicFunction};
use fasteval::{Instruction, Slab};
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
///   compatible for multiplication and will not check.

#[derive(Debug, Clone, PartialEq, Default)]
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

    pub fn from_vec(matrix: &Vec<Vec<ExprScalar>>) -> Self {
        ExprMatrix {
            matrix: matrix.to_owned(),
        }
    }

    /// returns dimensions of symbolic matrix.
    /// NOTE: a matrix [[]] will have (0,0) dimensions
    pub fn n_dims(&self) -> (usize, usize) {
        match self.matrix.first() {
            Some(row) if !row.is_empty() => (self.matrix.len(), row.len()),
            _ => (0, 0),
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
    pub fn zeros(dims: (usize, usize)) -> Self {
        let (rows, cols) = dims;
        let zero_matrix = vec![vec![ExprScalar::new("0"); cols]; rows];
        ExprMatrix {
            matrix: zero_matrix,
        }
    }

    pub fn hstack(mat: &[ExprMatrix]) -> Self {
        let mut result = Vec::new();
        let n_rows = mat.first().map_or(0, |m| m.n_dims().0);

        for row_idx in 0..n_rows {
            let mut new_row = Vec::new();
            for matrix in mat {
                if row_idx < matrix.matrix.len() {
                    new_row.extend(matrix.matrix[row_idx].clone());
                }
            }
            result.push(new_row);
        }

        ExprMatrix { matrix: result }
    }
    pub fn vstack(mat: &[ExprMatrix]) -> Self {
        let mut result = Vec::new();
        for matrix in mat {
            result.extend(matrix.matrix.clone());
        }
        ExprMatrix { matrix: result }
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

    pub fn sub(&self, other: &Self) -> Self {
        let result_matrix = self
            .matrix
            .iter()
            .zip(&other.matrix)
            .map(|(row_self, row_other)| {
                row_self
                    .iter()
                    .zip(row_other)
                    .map(|(val_self, val_other)| val_self.sub(val_other))
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

    pub fn wrap(&self) -> Self {
        let wrapped_matrix: Vec<Vec<_>> = self
            .matrix
            .iter()
            .map(|row| row.iter().map(|el| el.wrap()).collect())
            .collect();

        ExprMatrix {
            matrix: wrapped_matrix,
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

    /// mutliplies matrix by vector, and returns vector
    pub fn matmul_vec(&self, other: &ExprVector) -> ExprVector {
        let result_vector = self
            .matrix
            .iter()
            .map(|row| {
                row.iter()
                    .zip(&other.to_vec())
                    .map(|(matrix_elem, vector_elem)| matrix_elem.mul(vector_elem))
                    .fold(ExprScalar::default(), |acc, val| acc.add(&val))
            })
            .collect();
        ExprVector::from_vec(result_vector)
    }

    pub fn identity(dims: usize) -> Self {
        let mut identity_matrix = vec![vec![ExprScalar::default(); dims]; dims];

        for (i, row) in identity_matrix.iter_mut().enumerate().take(dims) {
            row[i] = ExprScalar::new("1");
        }

        ExprMatrix {
            matrix: identity_matrix,
        }
    }

    /// scale matrix my scalar expression
    pub fn scale(&self, var: &ExprScalar) -> ExprMatrix {
        let scaled_matrix = self
            .matrix
            .iter()
            .map(|row| row.iter().map(|elem| elem.scale(var)).collect())
            .collect();
        ExprMatrix {
            matrix: scaled_matrix,
        }
    }

    /// scale matrix by f64
    pub fn scalef(&self, var: f64) -> ExprMatrix {
        let scaled_matrix = self
            .matrix
            .iter()
            .map(|row| row.iter().map(|elem| elem.scalef(var)).collect())
            .collect();
        ExprMatrix {
            matrix: scaled_matrix,
        }
    }

    /// Builds the Karush-Kuhn-Tucker (KKT) Jacobian matrix from the Lagrangian Hessian
    /// and the equality constraint Jacobian. This matrix is used in optimization problems
    /// to solve systems of equations that arise in constrained optimization.
    ///
    /// # Arguments
    ///
    /// * `hessian` - A reference to the Lagrangian Hessian matrix, which represents the
    ///   second-order partial derivatives of the Lagrangian function with respect to the
    ///   optimization variables.
    /// * `jacobian` - A reference to the equality constraint Jacobian matrix, which
    ///   represents the first-order partial derivatives of the equality constraints with
    ///   respect to the optimization variables.
    /// * `regularization_factor` - A floating-point value used to scale the regularization
    ///   matrix. This factor is added to the diagonal of the Hessian to ensure numerical
    ///   stability and to regularize the system.
    ///
    /// # Returns
    ///
    /// An `ExprMatrix` representing the KKT Jacobian matrix. The structure of the matrix is as follows:
    /// - The top-left block contains the regularized Hessian matrix.
    /// - The top-right block contains the transpose of the Jacobian matrix.
    /// - The bottom-left block contains the Jacobian matrix.
    /// - The bottom-right block contains a scaled identity matrix with a negative sign.
    ///
    /// If there are no constraints (`n_constraints == 0`), the function returns a clone of the Hessian matrix.
    ///
    /// # Notes
    ///
    /// - The function assumes that the dimensions of the Hessian and Jacobian matrices are
    ///   compatible for constructing the KKT Jacobian matrix.
    /// - The regularization factor is applied to the diagonal of the Hessian to improve
    ///   numerical stability, especially in cases where the Hessian is ill-conditioned.
    pub fn build_ktt_jacobian(
        hessian: &ExprMatrix,
        jacobian: &ExprMatrix,
        regularization_factor: f64,
    ) -> ExprMatrix {
        let (n_constraints, _) = jacobian.n_dims();
        let (n_unknowns, _) = hessian.n_dims();

        // Top-left: H + εI
        let reg_hessian =
            hessian.add(&ExprMatrix::identity(n_unknowns).scalef(regularization_factor));

        // Shortcut if unconstrained
        if n_constraints == 0 {
            return reg_hessian.clone();
        }

        // Top-right: Jᵀ
        let jt = jacobian.transpose();

        // Bottom-left: J
        let j = jacobian.clone();

        // Bottom-right: -εI (for stabilization)
        let minus_reg = ExprMatrix::identity(n_constraints).scalef(-regularization_factor);

        // Horizontally stack top and bottom blocks
        let top = ExprMatrix::hstack(&[reg_hessian, jt]);
        let bottom = ExprMatrix::hstack(&[j, minus_reg]);

        // Final vertical stack
        ExprMatrix::vstack(&[top, bottom])
    }

    pub fn build_ip_ktt_jacobian(
        hessian: &ExprMatrix,
        eq_jacobian: &ExprMatrix,
        ineq_jacobian: &ExprMatrix,
        lambda: ExprMatrix,
        neg_s: ExprMatrix,
        regularization_factor: f64,
    ) -> ExprMatrix {
        let (n_vars, _) = hessian.n_dims();
        let (n_eq, _) = eq_jacobian.n_dims();
        let (n_ineq, _) = ineq_jacobian.n_dims();

        let reg_hessian = hessian.add(&ExprMatrix::identity(n_vars).scalef(regularization_factor));
        let g_scaled = ineq_jacobian.transpose().matmul(&lambda);

        let top = if n_eq > 0 {
            ExprMatrix::hstack(&[reg_hessian, eq_jacobian.transpose(), g_scaled])
        } else {
            ExprMatrix::hstack(&[reg_hessian, g_scaled])
        };

        let middle = if n_eq > 0 {
            Some(ExprMatrix::hstack(&[
                eq_jacobian.clone(),
                ExprMatrix::zeros((n_eq, n_eq)),
                ExprMatrix::zeros((n_eq, n_ineq)),
            ]))
        } else {
            None
        };

        let bottom = if n_eq > 0 {
            ExprMatrix::hstack(&[
                ineq_jacobian.clone(),
                ExprMatrix::zeros((n_ineq, n_eq)),
                neg_s,
            ])
        } else {
            ExprMatrix::hstack(&[ineq_jacobian.clone(), neg_s])
        };

        if n_eq > 0 {
            ExprMatrix::vstack(&[top, middle.unwrap(), bottom])
        } else {
            ExprMatrix::vstack(&[top, bottom])
        }
    }

    pub fn get_slab(&self) -> Result<InstructionSlab, SymbolicError> {
        let compiled_expr: Result<Vec<Vec<(Instruction, Slab)>>, SymbolicError> = self
            .matrix
            .iter()
            .map(|row| {
                row.iter()
                    .map(|e| e.compile_with_retry())
                    .collect::<Result<Vec<_>, _>>() // collect inner row
            })
            .collect(); // 

        let compiled_expr = compiled_expr?;

        Ok(InstructionSlab::Matrix(compiled_expr))
    }

    pub fn get_slab_and_symbolic_fn(
        &self,
        symbols: &ExprVector,
        registry: &Arc<ExprRegistry>,
    ) -> Result<(InstructionSlab, SymbolicFunction), SymbolicError> {
        let compiled_expr = self.get_slab()?;
        let func = SymbolicFunction::new(self.to_fn(&registry)?, symbols);
        Ok((compiled_expr, func))
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

    fn to_fn(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError> {
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
                    match f(vars_opt)? {
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
    use crate::numeric_services::symbolic::SymbolicExpr;
    use crate::numeric_services::symbolic::fasteval::ExprRegistry;
    use crate::numeric_services::symbolic::fasteval::utils::*;
    use rand::Rng;

    use nalgebra::{DMatrix, DVector};
    use proptest::prelude::*;

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
        assert_eq!(result.matrix[0][0].to_string(), "0 + 1 * 5 + 2 * 7");
        assert_eq!(result.matrix[0][1].to_string(), "0 + 1 * 6 + 2 * 8");
        assert_eq!(result.matrix[1][0].to_string(), "0 + 3 * 5 + 4 * 7");
        assert_eq!(result.matrix[1][1].to_string(), "0 + 3 * 6 + 4 * 8");
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

    #[test]
    fn test_from_string() {
        let input = vec![
            vec!["1".to_string(), "2".to_string()],
            vec!["3".to_string(), "4".to_string()],
        ];
        let expr_matrix = ExprMatrix::from_string(&input);
        assert_eq!(expr_matrix.matrix.len(), 2);
        assert_eq!(expr_matrix.matrix[0].len(), 2);
        assert_eq!(expr_matrix.matrix[0][0].to_string(), "1");
        assert_eq!(expr_matrix.matrix[1][1].to_string(), "4");
    }

    #[test]
    fn test_hstack() {
        let input1 = vec![
            vec!["1".to_string(), "2".to_string()],
            vec!["3".to_string(), "4".to_string()],
        ];
        let input2 = vec![
            vec!["5".to_string(), "6".to_string()],
            vec!["7".to_string(), "8".to_string()],
        ];
        let expr_matrix1 = ExprMatrix::from_string(&input1);
        let expr_matrix2 = ExprMatrix::from_string(&input2);

        let r = ExprMatrix::hstack(&[expr_matrix1, expr_matrix2]);
        assert_eq!(r.matrix.len(), 2);
        assert_eq!(r.matrix[0].len(), 4);
        assert_eq!(r.matrix[0][0].to_string(), "1");
        assert_eq!(r.matrix[0][2].to_string(), "5");
        assert_eq!(r.matrix[1][1].to_string(), "4");
        assert_eq!(r.matrix[1][2].to_string(), "7");
    }

    #[test]
    fn test_matmul_vec() {
        let matrix = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        let vector = ExprVector::from_vec(vec![ExprScalar::new("5"), ExprScalar::new("6")]);
        let result = matrix.matmul_vec(&vector);
        let result_vec = result.to_vec();
        assert_eq!(result_vec[0].to_string(), "0 + 1 * 5 + 2 * 6");
        assert_eq!(result_vec[1].to_string(), "0 + 3 * 5 + 4 * 6");
    }

    #[test]
    fn test_display() {
        let matrix = ExprMatrix::new(&vec![&["1", "2"], &["3", "4"]]);
        let display_output = format!("{}", matrix);
        assert_eq!(display_output, "[[1, 2], [3, 4]]");
    }

    #[test]
    fn test_empty_matrix() {
        let empty_matrix: Vec<&[&str]> = vec![];
        let expr_matrix = ExprMatrix::new(&empty_matrix);
        assert_eq!(expr_matrix.matrix.len(), 0);
    }

    fn random_dmatrix(rows: usize, cols: usize) -> DMatrix<f64> {
        let mut rng = rand::thread_rng();
        let data: Vec<f64> = (0..rows * cols).map(|_| rng.gen_range(-1.0..1.0)).collect();
        DMatrix::from_vec(rows, cols, data)
    }
    fn random_dvector(rows: usize) -> DVector<f64> {
        let mut rng = rand::thread_rng();
        let data: Vec<f64> = (0..rows).map(|_| rng.gen_range(-1.0..1.0)).collect();
        DVector::from_vec(data)
    }

    proptest! {
        #[test]
        fn test_add_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m1 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m2 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m1_expr = from_dmatrix(m1.clone());
            let m2_expr = from_dmatrix(m2.clone());

            let m_target = m1 + m2;
            let m_expr = m1_expr.add(&m2_expr);

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Matrix(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }

    proptest! {
        #[test]
        fn test_sub_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m1 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m2 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m1_expr = from_dmatrix(m1.clone());
            let m2_expr = from_dmatrix(m2.clone());

            let m_target = m1 - m2;
            let m_expr = m1_expr.sub(&m2_expr);

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Matrix(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }

    proptest! {
        #[test]
        fn test_matmul_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m1 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m2 = random_dmatrix(n_cols as usize, n_rows as usize);
            let m1_expr = from_dmatrix(m1.clone());
            let m2_expr = from_dmatrix(m2.clone());

            let m_target = m1 * m2;
            let m_expr = m1_expr.matmul(&m2_expr);

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Matrix(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }

    proptest! {
        #[test]
        fn test_transpose_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m1 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m1_expr = from_dmatrix(m1.clone());

            let m_target = m1.transpose();
            let m_expr = m1_expr.transpose();

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Matrix(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }

    proptest! {
        #[test]
        fn test_matmul_vec_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m = random_dmatrix(n_rows as usize, n_cols as usize);
            let v = random_dvector(n_cols as usize);
            let m_expr = from_dmatrix(m.clone());
            let v_expr = from_dvector(v.clone());

            let m_target = m * v;
            let m_expr = m_expr.matmul_vec(&v_expr);

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Vector(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }

    proptest! {
        #[test]
        fn test_combined_arithmetic(
            n_cols in 1..10,
            n_rows in 1..10,
            scalar1 in -1.0..1.0,
            scalar2 in -1.0..1.0,
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let m1 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m2 = random_dmatrix(n_rows as usize, n_cols as usize);
            let m3 = random_dmatrix(n_rows as usize, n_cols as usize);

            let ma = random_dmatrix(n_cols as usize, n_rows as usize);
            let mb = random_dmatrix(n_cols as usize, n_rows as usize);
            let mc = random_dmatrix(n_cols as usize, n_rows as usize);

            let v1 = random_dvector(n_rows as usize);
            let v2 = random_dvector(n_rows as usize);
            let v3 = random_dvector(n_rows as usize);

            let m1_expr = from_dmatrix(m1.clone());
            let m2_expr = from_dmatrix(m2.clone());
            let m3_expr = from_dmatrix(m3.clone());

            let ma_expr = from_dmatrix(ma.clone());
            let mb_expr = from_dmatrix(mb.clone());
            let mc_expr = from_dmatrix(mc.clone());

            let v1_expr = from_dvector(v1.clone());
            let v2_expr = from_dvector(v2.clone());
            let v3_expr = from_dvector(v3.clone());

            let m_target = (scalar1 * (m1 + m2 -m3) * scalar2 * (ma + mb -mc)) * (v1 + v2 - v3);
            let m_expr1 = m1_expr.add(&m2_expr).sub(&m3_expr).wrap().scalef(scalar1);
            let m_expr2 = ma_expr.add(&mb_expr).sub(&mc_expr).wrap().scalef(scalar2);
            let v_expr = v1_expr.add(&v2_expr).sub(&v3_expr).wrap();
            let m_expr = m_expr1.matmul(&m_expr2).wrap().matmul_vec(&v_expr);

            let m_obtained = m_expr.to_fn(&registry).unwrap();

            if let Ok(SymbolicEvalResult::Vector(result)) = m_obtained(None) {
                assert!((m_target - result).abs().sum() < 1e-3);
            } else {
                panic!("Unexpected result");
            }
        }
    }
}
