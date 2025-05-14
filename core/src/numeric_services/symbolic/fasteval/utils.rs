use crate::numeric_services::symbolic::{ExprMatrix, ExprScalar, ExprVector};
use nalgebra::{DMatrix, DVector};

pub fn from_dvector(vector: DVector<f64>) -> ExprVector {
    let expr: Vec<_> = vector
        .iter()
        .map(|&val| ExprScalar::from_f64(val))
        .collect();
    ExprVector::from_vec(expr)
}
pub fn from_dmatrix(matrix: DMatrix<f64>) -> ExprMatrix {
    let rows = matrix.nrows();
    let cols = matrix.ncols();
    let mut expr_matrix = Vec::new();

    for i in 0..rows {
        let mut row = Vec::new();
        for j in 0..cols {
            row.push(ExprScalar::new(matrix[(i, j)].to_string()));
        }
        expr_matrix.push(row);
    }

    ExprMatrix::from_vec(&expr_matrix)
}
