use nalgebra::DMatrix;

// Build block diagonal matrix from a Vec of matrices
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

/// Kronecker product
/// If A is an m×n matrix and B is a p×q matrix, then C will be an mp×nq matrix.
//  Each element A[i,j] is multiplied by the entire matrix B, and the results are placed in a block structure.
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

pub fn dmat_to_vec(mat: &DMatrix<f64>) -> Vec<Vec<f64>> {
    (0..mat.nrows())
        .map(|i| mat.row(i).iter().cloned().collect())
        .collect()
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
