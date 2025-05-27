use nalgebra::DMatrix;

use crate::physics::ModelError;

use super::options::RicattiLQROptions;

pub(super) fn ricatti_recursion(
    a_mat: &DMatrix<f64>,
    b_mat: &DMatrix<f64>,
    q_mat: &DMatrix<f64>,
    r_mat: &DMatrix<f64>,
    p_next_mat: &DMatrix<f64>,
) -> Result<(DMatrix<f64>, DMatrix<f64>), ModelError> {
    // Bᵀ * P * A
    let bt_p = b_mat.transpose() * p_next_mat;
    let rhs = &bt_p * a_mat;

    // R + Bᵀ * P * B
    let lhs = r_mat + &bt_p * b_mat;

    // Try Cholesky solve first
    let k = if let Some(chol) = lhs.clone().cholesky() {
        chol.solve(&rhs)
    } else {
        // Fall back to LU if Cholesky fails (e.g., not positive definite)
        lhs.lu()
            .solve(&rhs)
            .ok_or(ModelError::EvaluationError)?
    };

    // Compute new P_k
    let p = q_mat + a_mat.transpose() * p_next_mat * (a_mat - b_mat * &k);

    Ok((p, k))
}

pub(crate) fn solve_steady_state_lqr(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    options: &RicattiLQROptions,
) -> Result<DMatrix<f64>, ModelError> {
    let mut p = q.clone(); // Start with Q
    let max_iter = options.get_max_iter();
    let tol = options.get_tol();

    for _ in 0..max_iter {
        let (p_new, k_new) = ricatti_recursion(a, b, q, r, &p)?;

        if (&p_new - &p).amax() < tol {
            return Ok(k_new);
        }

        p = p_new;
    }

    Err(ModelError::EvaluationError)
}
