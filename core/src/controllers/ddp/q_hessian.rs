use general::matrix::{self, compute_comm_kron_product};
use nalgebra::{DMatrix, DVector, SymmetricEigen};

use crate::controllers::ddp::utils;

const BETA_REG_COEFF: f64 = 0.1;
const BETA_REG_UPDATE_FACTOR: f64 = 2.0;

#[derive(Debug, Clone)]
pub(super) struct Q {
    q_xx: DMatrix<f64>,
    q_xu: DMatrix<f64>,
    q_ux: DMatrix<f64>,
    q_uu: DMatrix<f64>,
}

impl Q {
    pub(super) fn new(
        q_xx: DMatrix<f64>,
        q_xu: DMatrix<f64>,
        q_ux: DMatrix<f64>,
        q_uu: DMatrix<f64>,
    ) -> Self {
        Self {
            q_xx,
            q_xu,
            q_ux,
            q_uu,
        }
    }

    pub(super) fn from_second_order_terms(
        a_x: DMatrix<f64>,
        a_u: DMatrix<f64>,
        b_x: DMatrix<f64>,
        b_u: DMatrix<f64>,
        linear_cost: &DVector<f64>,
    ) -> Self {
        let nx = a_x.ncols();
        let nu = b_u.ncols();

        let q_xx = compute_comm_kron_product(linear_cost, &a_x, nx);
        let q_uu = compute_comm_kron_product(linear_cost, &b_u, nu);
        let q_ux = compute_comm_kron_product(linear_cost, &b_x, nu);
        let q_xu = compute_comm_kron_product(linear_cost, &a_u, nx);

        Q::new(q_xx, q_xu, q_ux, q_uu)
    }

    /// Regularizes the Q-function Hessian blocks to ensure positive definiteness.
    ///
    /// This method checks whether the full Q-function Hessian (composed of `q_xx`, `q_xu`, `q_ux`, and `q_uu`)
    /// is symmetric positive definite by forming the full matrix:
    ///
    ///     [ q_xx   q_xu ]
    ///     [ q_ux   q_uu ]
    ///
    /// If the matrix is not positive definite (i.e., it has non-positive eigenvalues),
    /// the function adds a regularization term `β * I` to the diagonal of `q_xx` and `q_uu`,
    /// and increases `β` geometrically until the full matrix becomes positive definite.
    ///
    /// This ensures the backward pass remains numerically stable and the gains can be computed reliably.
    ///
    /// Regularization parameters:
    /// - `BETA_REG_COEFF`: Initial regularization strength
    /// - `BETA_REG_UPDATE_FACTOR`: Multiplicative increase factor for `β`
    pub(super) fn regularize(&mut self) {
        let nx = self.q_xx.nrows();
        let nu = self.q_uu.nrows();
        let (q_xx, q_xu, q_ux, q_uu) = (&mut self.q_xx, &self.q_xu, &self.q_ux, &mut self.q_uu);

        let mut beta = BETA_REG_COEFF;
        loop {
            // Construct symmetric block matrix:
            // [Gxx  Gxu]
            // [Gux  Guu]
            let mut big_mat = DMatrix::<f64>::zeros(nx + nu, nx + nu);

            big_mat.view_mut((0, 0), (nx, nx)).copy_from(q_xx);
            big_mat.view_mut((0, nx), (nx, nu)).copy_from(q_xu);
            big_mat.view_mut((nx, 0), (nu, nx)).copy_from(q_ux);
            big_mat.view_mut((nx, nx), (nu, nu)).copy_from(q_uu);

            // Check if symmetric positive definite
            // Here we check if all eigenvalues > 0
            let eig = SymmetricEigen::new(big_mat.clone());
            if eig.eigenvalues.iter().all(|&ev| ev > 0.0) {
                break;
            }

            // Not positive definite: regularize Gxx and Guu by adding beta * I
            for i in 0..nx {
                q_xx[(i, i)] += beta;
            }
            for i in 0..nu {
                q_uu[(i, i)] += beta;
            }

            beta *= BETA_REG_UPDATE_FACTOR;
        }
    }

    pub(super) fn q_uu(&self) -> &DMatrix<f64> {
        &self.q_uu
    }
    pub(super) fn q_ux(&self) -> &DMatrix<f64> {
        &self.q_ux
    }

    /// Computes the feedback gain and feedforward control for unconstrained iLQR.
    ///
    /// Solves the linear system assuming no control limits:
    ///
    /// - `K = -Q_uu⁻¹ * Q_ux`
    /// - `d = -Q_uu⁻¹ * q_u`
    ///
    /// This is the standard solution to the backward pass in unconstrained iLQR.
    ///
    /// # Arguments
    /// * `q_u` - Gradient of cost-to-go with respect to control
    ///
    /// # Returns
    /// A tuple `(K, d)` where:
    /// - `K` is the feedback gain matrix
    /// - `d` is the feedforward control vector
    pub (super) fn compute_unconstrained_gains(&mut self, q_u: &DVector<f64>) -> (DMatrix<f64>, DVector<f64>) {
        self.regularize();
        let lu = self.q_uu.clone().lu();
        let feedforward_control = lu.solve(q_u).expect("Matrix is singular!");
        let feedback_gain = lu.solve(&self.q_ux).expect("Matrix is singular!");

        (feedback_gain, feedforward_control)
    }

    pub(super) fn compute_gains_via_inverse(
        &self,
        q_u: &DVector<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let q_uu_inv = matrix::invert(self.q_uu.clone());
        let feedforward_control = &q_uu_inv * q_u;
        let feedback_gain = &q_uu_inv * &self.q_ux;

        (feedback_gain, feedforward_control)
    }

    /// Computes the feedback gain matrix for constrained iLQR by removing active inputs.
    ///
    /// This function identifies which input dimensions are not saturated (the "free set"),
    /// and solves a reduced linear system over the unconstrained subset of controls:
    ///
    /// - `K[free] = -Q_uu[free, free]⁻¹ * Q_ux[free, :]`
    /// - `K[constrained] = 0`
    ///
    /// This approach allows applying feedback only where the QP solver allows freedom,
    /// avoiding destabilizing updates when some inputs are at hard bounds.
    ///
    /// # Arguments
    /// * `delta_u` - OSQP solution (feedforward control step)
    /// * `delta_lb` - Lower bound offset: `lb - u_nominal`
    /// * `delta_ub` - Upper bound offset: `ub - u_nominal`
    ///
    /// # Returns
    /// A full-sized feedback gain matrix `K`, with zero rows where controls are constrained.
    pub(super) fn compute_constrained_gains(
        &self,
        delta_u: &DVector<f64>,
        delta_lb: &DVector<f64>,
        delta_ub: &DVector<f64>,
    ) -> DMatrix<f64> {
        let free_indices = utils::find_free_set(delta_u, delta_lb, delta_ub);
        let quu = self.q_uu();
        let qux = self.q_ux();

        let n_x = qux.ncols();
        let n_u = quu.nrows();

        let mut quu_free = DMatrix::<f64>::zeros(free_indices.len(), free_indices.len());
        let mut qux_free = DMatrix::<f64>::zeros(free_indices.len(), n_x);

        if free_indices.is_empty() {
            return DMatrix::<f64>::zeros(n_u, n_x);
        }


        for (i_f, &i) in free_indices.iter().enumerate() {
            for (j_f, &j) in free_indices.iter().enumerate() {
                quu_free[(i_f, j_f)] = quu[(i, j)];
            }
            for j in 0..n_x {
                qux_free[(i_f, j)] = qux[(i, j)];
            }
        }
        let lambda = 1e-6;
        for i in 0..quu_free.nrows() {
            quu_free[(i, i)] += lambda;
        }

        let k_free = quu_free.lu().solve(&qux_free).unwrap();

        let mut feedback_gain = DMatrix::<f64>::zeros(n_u, n_x);
        for (i_f, &i) in free_indices.iter().enumerate() {
            feedback_gain.set_row(i, &k_free.row(i_f));
        }
        feedback_gain
    }

    pub(super) fn compute_cost_terms(
        &self,
        q_x: &DVector<f64>,
        q_u: &DVector<f64>,
        feedback_gain: &DMatrix<f64>,
        feedforward_control: &DVector<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let feedback_gain_t = &feedback_gain.transpose();
        let quadratic_term = &self.q_xx + feedback_gain_t * &self.q_uu * feedback_gain
            - &self.q_xu * feedback_gain
            - feedback_gain_t * &self.q_ux;
        let linear_term = q_x - feedback_gain_t * q_u
            + feedback_gain_t * &self.q_uu * feedforward_control
            - &self.q_xu * feedforward_control;

        (quadratic_term, linear_term)
    }
}

impl std::ops::Add for Q {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        let q_xx = &self.q_xx + rhs.q_xx;
        let q_xu = &self.q_xu + rhs.q_xu;
        let q_ux = &self.q_ux + rhs.q_ux;
        let q_uu = &self.q_uu + rhs.q_uu;
        Q::new(q_xx, q_xu, q_ux, q_uu)
    }
}
