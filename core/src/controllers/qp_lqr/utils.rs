use crate::controllers::{ControllerInput, ControllerState, CostFn};
use crate::physics::traits::{PhysicsSim, State};
use crate::utils::matrix;
use nalgebra::{DMatrix, DVector};
use osqp::CscMatrix;

/// d = [-A*x0; zeros(size(C,1)-n)]
pub(super) fn build_d(x0: DVector<f64>, a: &DMatrix<f64>, c: usize) -> DVector<f64> {
    let n = a.nrows();
    let top = -a * x0;
    let bottom = DVector::<f64>::zeros(c - n);
    let mut d = DVector::<f64>::zeros(c);
    d.rows_mut(0, n).copy_from(&top);
    d.rows_mut(n, c - n).copy_from(&bottom);

    d
}

/// C = [[B1, -I, 0......0],[0, A2, B1, -I, 0,....0], [0....0 An-1, Bn-1, -I]]
pub(super) fn build_c(a: &DMatrix<f64>, b: &DMatrix<f64>, n_steps: usize) -> DMatrix<f64> {
    let n = a.nrows();
    let m = b.ncols();

    // Each constraint row corresponds to:
    // x_{k+1} - A x_k - B u_k = 0
    // Form: [ ... B  -I  ... ] with A added manually

    // Build block: [B  -I]
    let mut b_minus_i = DMatrix::<f64>::zeros(n, m + n);
    b_minus_i.view_mut((0, 0), (n, m)).copy_from(b);
    b_minus_i
        .view_mut((0, m), (n, n))
        .copy_from(&-DMatrix::<f64>::identity(n, n));

    // Kronecker product: kron(I(N-1), [B -I])
    let kron_block = matrix::kron(
        &DMatrix::<f64>::identity(n_steps, n_steps),
        &b_minus_i,
    );

    let mut c = kron_block;

    // Insert A blocks into the correct offsets
    for k in 0..(n_steps - 1) {
        let row_start = k * n;
        let col_start = k * (n + m) + m;

        c.view_mut((row_start, col_start), (n, n)).copy_from(a);
    }

    c
}

pub(super) fn build_h<'a, S: PhysicsSim>(
    cost_fn: &CostFn<S>,
    state_dim: usize,
    input_dim: usize,
    n: usize,
) -> CscMatrix<'a> {
    let r_mat = cost_fn
        .get_r()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(input_dim, input_dim));
    let q_mat = cost_fn
        .get_q()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(state_dim, state_dim));
    let qn_mat = cost_fn
        .get_qn()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(state_dim, state_dim));

    let z_dim = n * (input_dim + state_dim); // total size of z
    let mut h = DMatrix::<f64>::zeros(z_dim, z_dim);

    for k in 0..n {
        let u_start = k * (input_dim + state_dim);
        let x_start = u_start + input_dim;

        // Apply R to u_k
        h.view_mut((u_start, u_start), (input_dim, input_dim))
            .copy_from(&r_mat);

        // Apply Q to x_{k+1}, or QN at final step
        let qk = if k == n - 1 { &qn_mat } else { &q_mat };
        h.view_mut((x_start, x_start), (state_dim, state_dim))
            .copy_from(qk);
    }

    let h_vec = matrix::dmat_to_vec(&h);

    CscMatrix::from(&h_vec)
}

/// Builds G such that G * z >= h (input constraints only)
pub fn build_g(input_dim: usize, state_dim: usize, n_steps: usize) -> DMatrix<f64> {
    let total_vars = (n_steps - 1) * (input_dim + state_dim); // length of z
    let g_rows = 2 * input_dim * (n_steps - 1); // lower and upper bounds

    let mut g = DMatrix::<f64>::zeros(g_rows, total_vars);

    for k in 0..(n_steps - 1) {
        let u_start = k * (input_dim + state_dim); // position of u_k in z

        for i in 0..input_dim {
            let row_lower = k * input_dim + i;
            let row_upper = (n_steps - 1) * input_dim + row_lower;

            // Lower bound: -u_k[i] ≤ -u_min[i]
            g[(row_lower, u_start + i)] = -1.0;

            // Upper bound:  u_k[i] ≤ u_max[i]
            g[(row_upper, u_start + i)] = 1.0;
        }
    }

    g
}

pub fn build_h_vec(u_min: &DVector<f64>, u_max: &DVector<f64>, n_steps: usize) -> DVector<f64> {
    let input_dim = u_min.len();
    let mut h_vec = DVector::<f64>::zeros(2 * input_dim * (n_steps - 1));

    for k in 0..(n_steps - 1) {
        for i in 0..input_dim {
            let idx_lower = k * input_dim + i;
            let idx_upper = (n_steps - 1) * input_dim + idx_lower;

            h_vec[idx_lower] = -u_max[i]; // For -u_k ≤ -u_min
            //h_vec[idx_upper] = u_max[i];  // For  u_k ≤ u_max
            h_vec[idx_upper] = u_min[i]; // For  u_k ≤ u_max
        }
    }

    h_vec
}

pub fn build_q_vec<S: PhysicsSim>(
    cost_fn: &CostFn<S>,
    x_trajectory: &[ControllerState<S>],
    u_trajectory: &[ControllerInput<S>],
    n_steps: usize,
) -> DVector<f64> {
    let state_dims = ControllerState::<S>::dim_v() + ControllerState::<S>::dim_q();
    let input_dims = ControllerInput::<S>::dim_q();

    let r_mat = cost_fn
        .get_r()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(input_dims, input_dims));
    let q_mat = cost_fn
        .get_q()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(state_dims, state_dims));
    let qn_mat = cost_fn
        .get_qn()
        .cloned()
        .unwrap_or_else(|| DMatrix::zeros(state_dims, state_dims));

    let mut q = DVector::zeros((n_steps) * (state_dims + input_dims));
    let default_running_input_cost = -&r_mat * u_trajectory[0].to_vector();
    let default_running_state_cost = -&q_mat * x_trajectory[0].to_vector();

    //let offset = input_dims + (j - 1) * (state_dims + input_dims);
    //b_vec.rows_mut(offset, state_dims).copy_from(&q_x);
    for i in 0..n_steps - 1 {
        let offset_input = i * (state_dims + input_dims);
        let running_input_cost = if u_trajectory.len() == 1 {
            &default_running_input_cost
        } else {
            &(-&r_mat * u_trajectory[i].to_vector())
        };
        q.rows_mut(offset_input, input_dims)
            .copy_from(&running_input_cost);

        let offset_state = i * (state_dims + input_dims) + input_dims;
        let running_state_cost = if x_trajectory.len() == 1 {
            &default_running_state_cost
        } else {
            &(-&q_mat * x_trajectory[i].to_vector())
        };
        q.rows_mut(offset_state, state_dims)
            .copy_from(&running_state_cost);
    }
    let offset = (n_steps - 1) * (state_dims + input_dims) + input_dims;
    let terminal_cost = -qn_mat * x_trajectory.last().unwrap().to_vector();
    q.rows_mut(offset, state_dims).copy_from(&terminal_cost);

    q
}

#[cfg(test)]
mod tests {
    use crate::{
        cost::{CostFunction, generic::GenericCost},
        physics::{
            discretizer::ZOH,
            models::{LtiInput, LtiModel},
            simulator::BasicSim,
        },
    };

    use super::*;

    #[test]
    fn test_build_d() {
        let x0 = DVector::from_vec(vec![1.0, 2.0]);
        let a = DMatrix::from_vec(2, 2, vec![1.0, 0.0, 0.0, 1.0]);
        let c = 4;

        // d = [-A*x0; zeros(size(C,1)-n)]
        let d = build_d(x0, &a, c);

        assert_eq!(d.len(), c);
        assert_eq!(d[0], -1.0);
        assert_eq!(d[1], -2.0);
        assert_eq!(d[2], 0.0);
        assert_eq!(d[3], 0.0);
    }

    #[test]
    fn test_build_c() {
        let a = DMatrix::from_vec(2, 2, vec![1.0, 2.0, 2.0, 1.0]);
        let b = DMatrix::from_vec(2, 1, vec![1.0, 1.0]);
        let n_steps = 4;

        //  A = [[1,2],[2,1]], B = [[1],[1]],
        //  C is 6 x 9 ; [[B, -I, 0x6], [0, A, B, -I, 0x5], [0x2 A,B,I,0x4]....[0x6, A, B, -I]]

        let c = build_c(&a, &b, n_steps);

        assert_eq!(c.nrows(), 6);
        assert_eq!(c.ncols(), 9);
        assert_eq!(
            c.row(0),
            DMatrix::from_vec(1, 9, vec![1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        );
        assert_eq!(
            c.row(1),
            DMatrix::from_vec(1, 9, vec![1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        );
        assert_eq!(
            c.row(2),
            DMatrix::from_vec(1, 9, vec![0.0, 1.0, 2.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        );
        assert_eq!(
            c.row(3),
            DMatrix::from_vec(1, 9, vec![0.0, 2.0, 1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0])
        );
    }

    #[test]
    fn test_build_h() {
        let state_dim = 2;
        let input_dim = 1;
        let n = 4;

        let q_factor = 2.0;
        let qn_factor = 3.0;
        let r_factor = 0.1;

        let q_matrix = DMatrix::<f64>::identity(state_dim, state_dim) * q_factor;
        let qn_matrix = DMatrix::<f64>::identity(state_dim, state_dim) * qn_factor;
        let r_matrix = DMatrix::<f64>::identity(input_dim, input_dim) * r_factor;

        let cost: Box<dyn CostFunction<Input = _, State = _>> = Box::new(
            GenericCost::<_, LtiInput<1, 0>>::new(
                q_matrix.clone(),
                qn_matrix,
                r_matrix.clone(),
                None,
            )
            .unwrap(),
        );

        let h = build_h::<BasicSim<LtiModel<2, 0, 1>, ZOH<LtiModel<2, 0, 1>>>>(
            &cost, state_dim, input_dim, n,
        );

        assert_eq!(h.ncols, (n - 1) * (q_matrix.ncols() + r_matrix.ncols()));
        assert_eq!(h.nrows, (n - 1) * (q_matrix.nrows() + r_matrix.nrows()));
        assert_eq!(h.data[0], r_factor);
        assert_eq!(h.data[1], q_factor);
        assert_eq!(h.data[2], q_factor);
        assert_eq!(h.data[3], r_factor);
        assert_eq!(h.data[4], q_factor);
        assert_eq!(h.data[5], q_factor);
        assert_eq!(h.data[6], r_factor);
        assert_eq!(h.data[7], qn_factor);
        assert_eq!(h.data[8], qn_factor);

        // Add assertions to validate the structure of the resulting CscMatrix
    }
}
