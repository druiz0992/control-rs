use nalgebra::DMatrix;

/// Constructs a block diagonal matrix from a slice of matrices.
///
/// - **Parameters**:
///   - `blocks`: A slice of `DMatrix<f64>` representing the matrices to be placed in the block diagonal.
/// - **Returns**:
///   - A `DMatrix<f64>` representing the block diagonal matrix.
///
/// - **Example**:
/// ```rust
/// use nalgebra::dmatrix;
/// use control_rs::utils::matrix::block_diag;
///
/// let m1 = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let m2 = dmatrix![5.0, 6.0, 7.0; 8.0, 9.0, 10.0];
/// let result = block_diag(&[m1, m2]);
/// ```
pub fn block_diag(blocks: &[DMatrix<f64>]) -> DMatrix<f64> {
    let total_rows = blocks.iter().map(|m| m.nrows()).sum();
    let total_cols = blocks.iter().map(|m| m.ncols()).sum();
    let mut result = DMatrix::<f64>::zeros(total_rows, total_cols);

    let mut row = 0;
    let mut col = 0;
    for block in blocks {
        let (r, c) = (block.nrows(), block.ncols());
        result.view_mut((row, col), (r, c)).copy_from(block);
        row += r;
        col += c;
    }
    result
}

/// Computes the Kronecker product of two matrices.
///
/// - **Parameters**:
///   - `a`: A reference to a `DMatrix<f64>` representing the first matrix.
///   - `b`: A reference to a `DMatrix<f64>` representing the second matrix.
/// - **Returns**:
///   - A `DMatrix<f64>` representing the Kronecker product.
///      If A is an m×n matrix and B is a p×q matrix, then C will be an mp×nq matrix.
///       Each element A[i,j] is multiplied by the entire matrix B, and the results are placed in a block structure.
///
/// - **Example**:
/// ```rust
/// use nalgebra::dmatrix;
/// use control_rs::utils::matrix::kron;
///
/// let a = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let b = dmatrix![0.0, 5.0; 6.0, 7.0];
/// let result = kron(&a, &b);
/// ```
pub fn kron(a: &DMatrix<f64>, b: &DMatrix<f64>) -> DMatrix<f64> {
    let (ar, ac) = a.shape();
    let (br, bc) = b.shape();
    let mut result = DMatrix::<f64>::zeros(ar * br, ac * bc);

    for i in 0..ar {
        for j in 0..ac {
            let scalar = a[(i, j)];
            let sub = b * scalar;
            result.view_mut((i * br, j * bc), (br, bc)).copy_from(&sub);
        }
    }

    result
}

/// Converts a `DMatrix<f64>` into a `Vec<Vec<f64>>`.
///
/// - **Parameters**:
///   - `mat`: A reference to a `DMatrix<f64>` to be converted.
/// - **Returns**:
///   - A `Vec<Vec<f64>>` representing the matrix rows as vectors.
///
/// - **Example**:
/// ```rust
/// use nalgebra::dmatrix;
/// use control_rs::utils::matrix::dmat_to_vec;
///
/// let mat = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let vec = dmat_to_vec(&mat);
/// ```
pub fn dmat_to_vec(mat: &DMatrix<f64>) -> Vec<Vec<f64>> {
    (0..mat.nrows())
        .map(|i| mat.row(i).iter().cloned().collect())
        .collect()
}

/// Vertically stacks two matrices, where the second matrix is optional.
///
/// - **Parameters**:
///   - `a`: A `DMatrix<f64>` representing the first matrix.
///   - `b`: An `Option<DMatrix<f64>>` representing the second matrix.
/// - **Returns**:
///   - A `Result<DMatrix<f64>, &'static str>` containing the vertically stacked matrix or an error message if the column counts do not match.
///
/// - **Example**:
/// ```rust
/// use nalgebra::dmatrix;
/// use control_rs::utils::matrix::vstack_option;
///
/// let a = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let b = Some(dmatrix![5.0, 6.0; 7.0, 8.0]);
/// let result = vstack_option(a, b);
/// ```
pub fn vstack_option(
    a: DMatrix<f64>,
    b: Option<DMatrix<f64>>,
) -> Result<DMatrix<f64>, &'static str> {
    match b {
        Some(mat_b) => {
            let (cols_a, cols_b) = (a.ncols(), mat_b.ncols());
            if cols_a != cols_b {
                return Err("Matrix column counts must match for vertical stacking.");
            }

            let mut stacked = DMatrix::zeros(a.nrows() + mat_b.nrows(), cols_a);
            stacked.view_mut((0, 0), (a.nrows(), cols_a)).copy_from(&a);
            stacked
                .view_mut((a.nrows(), 0), (mat_b.nrows(), cols_b))
                .copy_from(&mat_b);
            Ok(stacked)
        }
        None => Ok(a),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::dmatrix;

    #[test]
    fn test_block_diag() {
        let m1 = dmatrix![
            1.0, 2.0;
            3.0, 4.0
        ];
        let m2 = dmatrix![
            5.0, 6.0, 7.0;
            8.0, 9.0, 10.0
        ];
        let m3 = dmatrix![11.0];

        let result = block_diag(&[m1.clone(), m2.clone(), m3.clone()]);
        let expected = dmatrix![
            1.0, 2.0, 0.0, 0.0, 0.0, 0.0;
            3.0, 4.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 0.0, 5.0, 6.0, 7.0, 0.0;
            0.0, 0.0, 8.0, 9.0, 10.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 0.0, 11.0
        ];

        assert_eq!(result, expected);
    }

    #[test]
    fn test_kron() {
        let a = dmatrix![
            1.0, 2.0;
            3.0, 4.0
        ];
        let b = dmatrix![
            0.0, 5.0;
            6.0, 7.0
        ];

        let result = kron(&a, &b);
        let expected = dmatrix![
            0.0, 5.0, 0.0, 10.0;
            6.0, 7.0, 12.0, 14.0;
            0.0, 15.0, 0.0, 20.0;
            18.0, 21.0, 24.0, 28.0
        ];

        assert_eq!(result, expected);
    }

    #[test]
    fn test_block_diag_empty() {
        let result = block_diag(&[]);
        let expected = DMatrix::<f64>::zeros(0, 0);
        assert_eq!(result, expected);
    }

    #[test]
    fn test_kron_with_zero_matrix() {
        let a = dmatrix![
            1.0, 2.0;
            3.0, 4.0
        ];
        let b = DMatrix::<f64>::zeros(2, 2);

        let result = kron(&a, &b);
        let expected = DMatrix::<f64>::zeros(4, 4);

        assert_eq!(result, expected);
    }
}
