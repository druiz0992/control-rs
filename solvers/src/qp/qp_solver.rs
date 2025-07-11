use crate::Minimizer;
use crate::SolverError;
use crate::dtos::{KktConditionsStatus, LagrangianMultiplier, OptimizerConfig, SolverResult};
use log::info;
use nalgebra::{DMatrix, DVector, DVectorView};

#[derive(Clone, Default)]
pub struct QP {
    pub(super) q_mat: DMatrix<f64>,
    pub(super) q_vec: DVector<f64>,
    pub(super) a_mat: DMatrix<f64>,
    pub(super) b_vec: DVector<f64>,
    pub(super) g_mat: DMatrix<f64>,
    pub(super) h_vec: DVector<f64>,

    pub(super) xi: std::ops::Range<usize>,
    pub(super) mui: std::ops::Range<usize>,
    pub(super) sigmai: std::ops::Range<usize>,

    pub(super) options: OptimizerConfig,
}

impl std::fmt::Debug for QP {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QP")
            .field("q_mat", &self.q_mat)
            .field("q_vec", &self.q_vec)
            .field("a_mat", &self.a_mat)
            .field("b_vec", &self.b_vec)
            .field("g_mat", &self.g_mat)
            .field("h_vec", &self.h_vec)
            .field("xi", &self.xi)
            .field("mui", &self.mui)
            .field("sigmai", &self.sigmai)
            .finish()
    }
}

impl QP {
    fn c_eq(&self, x: &DVectorView<f64>) -> DVector<f64> {
        &self.a_mat * x - &self.b_vec
    }

    fn h_ineq(&self, x: &DVectorView<f64>) -> DVector<f64> {
        &self.g_mat * x - &self.h_vec
    }

    fn kkt_conditions(
        &self,
        z: &DVector<f64>,
        rho: f64,
        status: &mut KktConditionsStatus,
    ) -> DVector<f64> {
        let x = z.rows(self.xi.start, self.xi.len());
        let mu = z.rows(self.mui.start, self.mui.len());
        let sigma = z.rows(self.sigmai.start, self.sigmai.len());

        let lambda = sigma.map(|s| rho.sqrt() * (-s).exp());

        let grad = &self.q_mat * x + &self.q_vec + self.a_mat.transpose() * mu
            - self.g_mat.transpose() * &lambda;

        let ceq = self.c_eq(&x);
        let h_vec = self.h_ineq(&x);

        let min_h = h_vec.map(|h| h.min(0.0));
        let min_l = lambda.map(|l| l.min(0.0));
        let comp = lambda.component_mul(&h_vec);

        *status = KktConditionsStatus {
            stationarity: grad.norm(),
            max_primal_feasibility_c: Some(ceq.amax()),
            min_primal_feasibility_h: Some(h_vec.min()),
            dual_feasibility: Some(lambda.amax()),
            complementary_slackness: Some(lambda.dot(&h_vec).abs()),
        };

        DVector::from_iterator(
            grad.len() + ceq.len() + min_h.len() + min_l.len() + comp.len(),
            grad.iter()
                .chain(ceq.iter())
                .chain(min_h.iter())
                .chain(min_l.iter())
                .chain(comp.iter())
                .copied(),
        )
    }

    fn ip_kkt_conditions(&self, z: &DVector<f64>, rho: f64) -> DVector<f64> {
        let x = z.rows(self.xi.start, self.xi.len());
        let mu = z.rows(self.mui.start, self.mui.len());
        let sigma = z.rows(self.sigmai.start, self.sigmai.len());

        let s = sigma.map(|s| rho.sqrt() * s.exp());
        let lambda = sigma.map(|s| rho.sqrt() * (-s).exp());

        let grad = &self.q_mat * x + &self.q_vec + self.a_mat.transpose() * mu
            - self.g_mat.transpose() * &lambda;

        let ceq = self.c_eq(&x);
        let h_vec = self.h_ineq(&x) - &s;

        DVector::from_iterator(
            grad.len() + ceq.len() + h_vec.len(),
            grad.iter().chain(ceq.iter()).chain(h_vec.iter()).copied(),
        )
    }

    fn ip_kkt_jacobian(&self, z: &DVector<f64>, rho: f64) -> DMatrix<f64> {
        let x_dim = self.xi.len();
        let n_eq = self.b_vec.len();
        let n_ineq = self.h_vec.len();

        let sigma = z.rows(self.sigmai.start, self.sigmai.len());
        let s_diag = DMatrix::from_diagonal(&sigma.map(|s| rho.sqrt() * s.exp()));
        let l_diag = DMatrix::from_diagonal(&sigma.map(|s| rho.sqrt() * (-s).exp()));

        let mut jac = DMatrix::zeros(x_dim + n_eq + n_ineq, x_dim + n_eq + n_ineq);

        jac.view_mut((0, 0), (x_dim, x_dim)).copy_from(&self.q_mat);
        jac.view_mut((0, x_dim), (x_dim, n_eq))
            .copy_from(&self.a_mat.transpose());
        jac.view_mut((0, x_dim + n_eq), (x_dim, n_ineq))
            .copy_from(&(self.g_mat.transpose() * &l_diag));

        jac.view_mut((x_dim, 0), (n_eq, x_dim))
            .copy_from(&self.a_mat);

        jac.view_mut((x_dim + n_eq, 0), (n_ineq, x_dim))
            .copy_from(&self.g_mat);
        jac.view_mut((x_dim + n_eq, x_dim + n_eq), (n_ineq, n_ineq))
            .copy_from(&(-s_diag));

        jac
    }

    pub fn solve_qp(&self, initial_guess: &[f64]) -> Result<SolverResult, SolverError> {
        let n = self.q_vec.len();
        let n_eq = self.b_vec.len();
        let n_ineq = self.h_vec.len();
        let mut z = DVector::zeros(n + n_eq + n_ineq);
        z.rows_mut(0, n).copy_from_slice(initial_guess);

        let mut rho = 0.1;
        let ls_options = self.options.get_line_search_opts();
        let mut status = KktConditionsStatus::default();

        for main_iter in 0..self.options.get_max_iters() {
            let res = self.ip_kkt_conditions(&z, rho);
            let jac = self.ip_kkt_jacobian(&z, rho);

            let dz = jac
                .lu()
                .solve(&(-&res))
                .ok_or_else(|| SolverError::Other("Failed to solve linear problem".into()))?;

            let mut alpha = 1.0;
            for _ in 0..ls_options.get_max_iters() {
                let z_new = &z + alpha * &dz;
                if self.ip_kkt_conditions(&z_new, rho).norm() < res.norm() {
                    break;
                }
                alpha *= ls_options.get_factor();
            }

            z += alpha * &dz;

            if self.options.get_verbose() {
                info!(
                    "iter: {}, kkt_status: {:?}, alpha: {}, rho: {}",
                    main_iter, &status, alpha, rho
                );
            }

            let kkt_result = self.kkt_conditions(&z, rho, &mut status);
            if kkt_result.amax() < self.options.get_tolerance() {
                let x = z.rows(self.xi.start, self.xi.len());
                let mu = z.rows(self.mui.start, self.mui.len());
                let sigma = z.rows(self.sigmai.start, self.sigmai.len());
                let lambda = sigma.map(|s| rho.sqrt() * (-s).exp());

                return Ok((
                    x.as_slice().to_vec(),
                    status,
                    LagrangianMultiplier::Mus(mu.as_slice().to_vec()),
                    LagrangianMultiplier::Lambdas(lambda.as_slice().to_vec()),
                ));
            } else if self.ip_kkt_conditions(&z, rho).amax() < self.options.get_tolerance() {
                rho *= 0.1;
            }
        }

        Err(SolverError::Other("QP Solver did not converge.".into()))
    }
}

impl Minimizer for QP {
    fn minimize(&self, initial_guess: &[f64]) -> Result<SolverResult, SolverError> {
        self.solve_qp(initial_guess)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{dmatrix, dvector};

    #[test]
    fn test_kkt_conditions() {
        let q_vec = dvector![-2.0, -5.0];
        let b_vec = dvector![1.0];
        let h_vec = dvector![0.0, 0.0];

        let kkt_conditions_len = q_vec.len() + b_vec.len() + 3 * h_vec.len();

        let qp = QP {
            q_mat: dmatrix![
                2.0, 0.0;
                0.0, 2.0
            ],
            q_vec,
            a_mat: dmatrix![1.0, 1.0],
            b_vec,
            g_mat: dmatrix![
                -1.0, 0.0;
                0.0, -1.0
            ],
            h_vec,
            xi: 0..2,
            mui: 2..3,
            sigmai: 3..5,
            options: OptimizerConfig::default(),
        };

        let z = dvector![0.5, 0.5, 0.0, 0.0, 0.0];
        let rho = 0.1;
        let mut status = KktConditionsStatus::default();

        let kkt_res = qp.kkt_conditions(&z, rho, &mut status);

        assert_eq!(kkt_res.len(), kkt_conditions_len);
    }

    #[test]
    fn test_ip_kkt_conditions() {
        let q_vec = dvector![-2.0, -5.0];
        let b_vec = dvector![1.0];
        let h_vec = dvector![0.0, 0.0];

        let ip_kkt_conditions_len = q_vec.len() + b_vec.len() + h_vec.len();
        let qp = QP {
            q_mat: dmatrix![
                2.0, 0.0;
                0.0, 2.0
            ],
            q_vec,
            a_mat: dmatrix![1.0, 1.0],
            b_vec,
            g_mat: dmatrix![
                -1.0, 0.0;
                0.0, -1.0
            ],
            h_vec,
            xi: 0..2,
            mui: 2..3,
            sigmai: 3..5,
            options: OptimizerConfig::default(),
        };

        let z = dvector![0.5, 0.5, 0.0, 0.0, 0.0];
        let rho = 0.1;

        let ip_kkt_res = qp.ip_kkt_conditions(&z, rho);
        assert_eq!(ip_kkt_res.len(), ip_kkt_conditions_len);
    }

    #[test]
    fn test_ip_kkt_jacobian() {
        let qp = QP {
            q_mat: dmatrix![
                2.0, 0.0;
                0.0, 2.0
            ],
            q_vec: dvector![-2.0, -5.0],
            a_mat: dmatrix![1.0, 1.0],
            b_vec: dvector![1.0],
            g_mat: dmatrix![
                -1.0, 0.0;
                0.0, -1.0
            ],
            h_vec: dvector![0.0, 0.0],
            xi: 0..2,
            mui: 2..3,
            sigmai: 3..5,
            options: OptimizerConfig::default(),
        };

        let ip_kkt_jacobian_cols = qp.q_mat.ncols() + qp.a_mat.nrows() + qp.g_mat.nrows();
        let ip_kkt_jacobian_rows = qp.q_mat.nrows() + qp.a_mat.nrows() + qp.g_mat.nrows();

        let z = dvector![0.5, 0.5, 0.0, 0.0, 0.0];
        let rho = 0.1;

        let jac = qp.ip_kkt_jacobian(&z, rho);
        assert_eq!(jac.nrows(), ip_kkt_jacobian_rows);
        assert_eq!(jac.ncols(), ip_kkt_jacobian_cols);
    }

    #[test]
    fn test_solve_qp() {
        let a_mat = dmatrix![1.0, 1.0];
        let b_vec = dvector![1.0];
        let g_mat = dmatrix![
            1.0, 0.0;
            0.0, 1.0
        ];
        let h_vec = dvector![0.0, 0.0];
        let qp = QP {
            q_mat: dmatrix![
                2.0, 0.0;
                0.0, 2.0
            ],
            q_vec: dvector![-2.0, -5.0],
            a_mat: a_mat.clone(),
            b_vec: b_vec.clone(),
            g_mat: g_mat.clone(),
            h_vec: h_vec.clone(),
            xi: 0..2,
            mui: 2..3,
            sigmai: 3..5,
            options: OptimizerConfig::default(),
        };

        let initial_guess = [0.5, 0.5];
        let result = qp.solve_qp(&initial_guess);

        assert!(result.is_ok());
        let (x, _status, _, _) = result.unwrap();
        let dv_x = DVector::from_vec(x);
        let tol = 1e-5;
        assert!((a_mat * &dv_x - b_vec).norm() <= tol);
        assert!((&g_mat * &dv_x - &h_vec).data.as_vec()[0] >= 0.0);
        assert!((&g_mat * &dv_x - &h_vec).data.as_vec()[1] >= 0.0);
    }
}
