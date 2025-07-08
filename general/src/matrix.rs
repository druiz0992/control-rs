use nalgebra::{DMatrix, DVector};

const LAMBDA_REG_COEFF: f64 = 1.0;

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
/// use general::matrix::block_diag;
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
/// use general::matrix::kron;
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
/// use general::matrix::dmat_to_vec;
///
/// let mat = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let vec = dmat_to_vec(&mat);
/// ```
pub fn dmat_to_vec(mat: &DMatrix<f64>) -> Vec<Vec<f64>> {
    (0..mat.nrows())
        .map(|i| mat.row(i).iter().cloned().collect())
        .collect()
}

pub fn vec_to_dmat(vec: &[Vec<f64>]) -> DMatrix<f64> {
    let rows = vec.len();
    let cols = if rows > 0 { vec[0].len() } else { 0 };
    let mut mat = DMatrix::<f64>::zeros(rows, cols);
    for (i, row) in vec.iter().enumerate() {
        for (j, val) in row.iter().enumerate() {
            mat[(i, j)] = *val;
        }
    }
    mat
}

/// Converts a matrix into a column vector by stacking its columns on top of each other (column-major order).
///
/// - **Parameters**:
///   - `mat`: A `DMatrix<f64>` to be vectorized.
/// - **Returns**:
///   - A `DVector<f64>` containing all elements of the matrix, stacked column by column.
///
/// - **Description**:
///   This function takes a matrix and returns a vector by concatenating its columns in order.
///   For a matrix of size (m, n), the resulting vector will have length m * n, with the first m elements
///   corresponding to the first column, the next m elements to the second column, and so on.
///
/// - **Example**:
/// ```rust
/// use nalgebra::{DVector,dmatrix};
/// use general::matrix::vectorize;
///
/// let mat = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let vec = vectorize(mat);
/// assert!(vec == DVector::from_vec(vec![1.0, 3.0, 2.0, 4.0]));
/// ```
pub fn vectorize(mat: DMatrix<f64>) -> DVector<f64> {
    let (rows, cols) = mat.shape();
    let mut data = Vec::with_capacity(rows * cols);
    for j in 0..cols {
        for i in 0..rows {
            data.push(mat[(i, j)]);
        }
    }
    DVector::from_vec(data)
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
/// use general::matrix::vstack_option;
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

/// Horizontally stacks two matrices.
///
/// - **Parameters**:
///   - `a`: A `DMatrix<f64>` representing the first matrix.
///   - `b`: A `DMatrix<f64>` representing the second matrix.
/// - **Returns**:
///   - A `Result<DMatrix<f64>, ModelError>` containing the horizontally stacked matrix or an error if the row counts do not match.
///
/// - **Example**:
/// ```rust
/// use nalgebra::dmatrix;
/// use general::matrix::hstack;
///
/// let a = dmatrix![1.0, 2.0; 3.0, 4.0];
/// let b = dmatrix![5.0; 6.0];
/// let result = hstack(a, b);
/// ```
pub fn hstack(a: DMatrix<f64>, b: DMatrix<f64>) -> Result<DMatrix<f64>, String> {
    if a.nrows() != b.nrows() {
        return Err("Mismatch in matrix dimensions.".into());
    }

    let mut stacked = DMatrix::zeros(a.nrows(), a.ncols() + b.ncols());
    stacked
        .view_mut((0, 0), (a.nrows(), a.ncols()))
        .copy_from(&a);
    stacked
        .view_mut((0, a.ncols()), (b.nrows(), b.ncols()))
        .copy_from(&b);

    Ok(stacked)
}


/// Efficiently computes the product:
///     kron(pᵀ, I(out_dim)) * comm(nx, out_dim) * X
///
/// This is equivalent to applying a Kronecker product between the transpose of `p`
/// and the identity matrix, followed by a commutator reshuffle, and finally a matrix
/// multiplication with `X`. Rather than explicitly forming the Kronecker or commutator
/// matrices, this function directly computes the result using indexing logic.
///
/// # Arguments
/// - `p`: A column vector of shape `(n,)`, representing the weights (typically from costate or dual vector).
/// - `m`: A matrix of shape `(nx * n, k)`, where each column represents a Jacobian flattened in row-major order.
/// - `nx`: The output dimension (number of rows in the result).
///
/// # Returns
/// A matrix of shape `(nx, k)`, representing the contracted and reshaped result.
pub fn compute_comm_kron_product(p: &DVector<f64>, m: &DMatrix<f64>, nx: usize) -> DMatrix<f64> {
    let n = p.len(); // Number of Jacobian blocks
    let k = m.ncols(); // Output dimensionality
    assert_eq!(m.nrows(), nx * n, "Matrix M should have {} rows", nx * n);

    let mut result = DMatrix::<f64>::zeros(nx, k);

    for i in 0..nx {
        for j in 0..k {
            for k_idx in 0..n {
                // M is flattened row-wise, so (i,k) is at (i * N + k)
                let row_idx = i * n + k_idx;
                result[(i, j)] += p[k_idx] * m[(row_idx, j)];
            }
        }
    }

    result
}

/// Applies the commutation operation K(n, m) * vec(A), without building K explicitly.
/// `vec` is the vectorized form of an n × m matrix (column-major).
/// Returns the vectorized form of Aᵗ (i.e., a DVector of length n * m).
pub fn apply_commutation(n: usize, m: usize, vec: &DVector<f64>) -> DVector<f64> {
    assert_eq!(vec.len(), n * m, "Vector size must be n * m");

    let a = DMatrix::from_column_slice(n, m, vec.as_slice());

    let a_t = a.transpose();
    DVector::from_column_slice(a_t.as_slice())
}

pub fn invert(q: DMatrix<f64>) -> DMatrix<f64> {
    let eig = q.symmetric_eigen();
    let mut evals = eig.eigenvalues;

    for i in 0..evals.len() {
        if evals[i] < 0.0 {
            evals[i] = 0.0;
        }
        evals[i] += LAMBDA_REG_COEFF;
    }

    let inv_evals = evals.map(|x| 1.0 / x);
    &eig.eigenvectors * DMatrix::from_diagonal(&inv_evals) * eig.eigenvectors.transpose()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{dmatrix, dvector};

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

    #[test]
    fn test_contract_vec_jacobian() {
        let nx = 4;
        let nu = 1;
        let p = dvector![
            -317.28345824511774,
            12.879506775484236,
            3.6319758270475235,
            -8.22339673224945
        ];
        let ax = dmatrix![
            -0.000524944348465173, -0.00017824939657732692, -1.9048959716537104e-5, -7.410490990425182e-6;
             0.00136830126533619, 0.00039554684967248996, 5.361993127907377e-5, 1.3571906304263965e-5;
            -0.020536263615741554, -0.007194287025401632, -0.0011262266456180845, -0.00044992060173964994;
             0.05365908885733304, 0.016123363005529814, 0.003168211608762129, 0.0008345227803683299;
            -0.00017824939657732697, -0.002324727185774613, 2.4598479149040764e-5, -9.731340628147828e-5;
             0.00039554684967248996, 0.005182701966305756, -5.7476747560828436e-5, 0.00021915692582767229;
            -0.007194287025401634, -0.08995885834151124, 0.0002505803851469757, -0.005353125149090376;
             0.016123363005529814, 0.2008056909520326, -0.0017835316712742821, 0.011386891225801513;
            -1.9048959716537104e-5, 2.459847914904077e-5, 0.0001859443861752974, 0.00012522531233387184;
             5.361993127907377e-5, -5.747674756082844e-5, -0.00043285938222003943, -0.00018777833269055904;
            -0.001126226645618084, 0.00025058038514697603, 0.0072964580189042606, 0.004921665343065639;
             0.003168211608762129, -0.0017835316712742823, -0.016959758920718014, -0.007400173780087397;
            -7.410490990425185e-6, -9.731340628147829e-5, 0.00012522531233387184, 0.0001191249711809489;
             1.3571906304263968e-5, 0.00021915692582767226, -0.00018777833269055904, -0.000174441990676855;
            -0.00044992060173965005, -0.005353125149090376, 0.004921665343065638, 0.004530754905739131;
             0.0008345227803683299, 0.011386891225801516, -0.007400173780087395, -0.006545905901531553];

        let au = dmatrix![
            7.104239273004845e-8;
            -4.0541713129360624e-7;
            4.9258900476925415e-6;
            -3.0230995841054384e-5;
            6.530017093582561e-5;
            -0.00014242284615599535;
            0.002483786518138624;
            -0.005421451329809722;
            1.0816262186979212e-6;
            -7.920282798137794e-8;
            6.428454324402658e-5;
            -6.202082543004191e-6;
            2.655423624944746e-6;
            -4.646788503128216e-6;
            0.00015240421149124438;
            -0.0002651263777990329
        ];

        let bx = dmatrix![
            7.104239273004846e-8, 6.530017093582561e-5, 1.0816262186979214e-6, 2.655423624944746e-6;
           -4.0541713129360624e-7, -0.00014242284615599535, -7.920282798137852e-8, -4.646788503128216e-6;
            4.9258900476925415e-6, 0.002483786518138624, 6.42845432440266e-5, 0.00015240421149124438;
           -3.0230995841054388e-5, -0.005421451329809722, -6.202082543004203e-6, -0.0002651263777990329];

        let bu = dmatrix![
            7.930922742546722e-8;
            -1.6710754000661293e-7;
            5.9952764606313425e-6;
            -1.2618552624347942e-5
        ];

        let expected_ax = dmatrix![
            -0.3316679853100629, -0.09706825391510789, -0.023409370860186303, -0.00597068698673934;
            -0.09706825391510787, -1.173685135647839, 0.007031827727385843, -0.07938277804770111;
            -0.023409370860186303, 0.0070318277273858434, 0.10139529200299936, 0.036579521970787676;
            -0.005970686986739339, -0.07938277804770114, 0.036579521970787655, 0.03024206387569549];

        let expected_au = dmatrix![
            0.00023873003725509567;
            0.031050797672466754;
            -5.972010812431235e-5;
            0.0018313974660636007
        ];

        let expected_bx = dmatrix![
            0.00023873003725509567,
            0.031050797672466754,
            -5.972010812431231e-5,
            0.0018313974660636007
        ];

        let expected_bu = dmatrix![9.82262949562102e-5];

        let result_ax = compute_comm_kron_product(&p, &ax, nx);

        let result_au = compute_comm_kron_product(&p, &au, nx);

        let result_bx = compute_comm_kron_product(&p, &bx, nu);

        let result_bu = compute_comm_kron_product(&p, &bu, nu);

        assert_eq!(result_ax, expected_ax);
        assert_eq!(result_au, expected_au);
        assert_eq!(result_bx, expected_bx);
        assert_eq!(result_bu, expected_bu);
    }
}

