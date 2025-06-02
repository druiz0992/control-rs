use super::solver::{OSQPSolver, OSQPSolverHandle};
use crate::{physics::ModelError, utils::matrix::dmat_to_vec};
use core::f64;
use nalgebra::{DMatrix, DVector};
use osqp::{CscMatrix, Problem, Settings};

type OSQPParams<'a> = (
    CscMatrix<'a>,
    Vec<f64>,
    CscMatrix<'a>,
    Vec<f64>,
    Vec<f64>,
    usize,
    usize,
);

/// minimize 1/2 x' P x  + q' x, st Ax=b, Gx>=h
#[derive(Default, Clone)]
pub struct QPParams<'a> {
    // objective function : 0.5 * x' * q_mat * x + q_vec' * x
    pub q_csc: Option<CscMatrix<'a>>,
    pub q_mat: Option<Vec<Vec<f64>>>,
    pub q_vec: Option<Vec<f64>>,
    // stacked  constraint matrix: lb <= a_mat * x <= ub
    pub a_mat: Option<DMatrix<f64>>,
    pub lb_vec: Option<DVector<f64>>,
    pub ub_vec: Option<DVector<f64>>,
}

#[derive(Clone, Default)]
pub struct OSQPBuilder<'a> {
    pub qp_params: QPParams<'a>,
    options: Settings,
}

impl<'a> OSQPBuilder<'a> {
    pub fn new() -> Self {
        Self {
            qp_params: QPParams::default(),
            options: Settings::default(),
        }
    }

    pub fn from_params(qp_params: QPParams<'a>) -> Self {
        Self {
            qp_params,
            options: Settings::default(),
        }
    }

    pub fn q_mat(mut self, q_mat: DMatrix<f64>) -> Self {
        let q = (0..q_mat.nrows())
            .map(|i| q_mat.row(i).iter().cloned().collect())
            .collect();
        self.qp_params.q_mat = Some(q);
        self
    }
    pub fn q_csc(mut self, q_csc: CscMatrix<'a>) -> Self {
        self.qp_params.q_csc = Some(q_csc);
        self
    }

    pub fn q_vec(mut self, q_vec: DVector<f64>) -> Self {
        self.qp_params.q_vec = Some(q_vec.as_slice().to_vec());
        self
    }

    pub fn a_mat(mut self, a_mat: DMatrix<f64>) -> Self {
        self.qp_params.a_mat = Some(a_mat);
        self
    }

    pub fn bounds_vec(mut self, lb_vec: DVector<f64>, ub_vec: DVector<f64>) -> Self {
        self.qp_params.lb_vec = Some(lb_vec);
        self.qp_params.ub_vec = Some(ub_vec);
        self
    }

    pub fn add_options(mut self, options: Settings) -> Self {
        self.options = options;
        self
    }

    pub fn build(self) -> Result<(OSQPSolverHandle, QPParams<'a>), ModelError> {
        self.check()?;
        let options = &self.options.clone();
        let qp_params = self.qp_params.clone();
        let (q_csc, q, a_csc, l, u, n_eq, n_ineq) = self.init();
        // check dimensions
        // q_mat, q_vec, a_mat, lb_vec, ub_vec
        // q_mat/q_csc is square
        if q_csc.ncols != q_csc.nrows {
            return Err(ModelError::ConfigError(
                "Q Matrix needs to be square.".into(),
            ))?;
        }

        if q_csc.nrows != q.len() {
            return Err(ModelError::ConfigError(
                "Q Matrix and q vector dimensions don't match.".into(),
            ))?;
        }
        if a_csc.ncols != q_csc.ncols {
            return Err(ModelError::ConfigError(
                "Incorrect number of columns for matrix A.".into(),
            ));
        }
        if l.len() != u.len() {
            return Err(ModelError::ConfigError(
                "Lower and upper bound vector dimensions don't match.".into(),
            ));
        }
        if l.len() != a_csc.nrows {
            return Err(ModelError::ConfigError(
                "Incorrect Lower bound vector dimensions.".into(),
            ));
        }
        if u.len() != a_csc.nrows {
            return Err(ModelError::ConfigError(
                "Incorrect Upper bound vector dimensions.".into(),
            ));
        }

        let solver = OSQPSolver {
            problem: Problem::new(q_csc, &q, a_csc, &l, &u, options)
                .map_err(|e| ModelError::ConfigError(e.to_string()))?,
            n_eq,
            n_ineq,
        };

        Ok((OSQPSolverHandle::new(solver), qp_params))
    }

    pub fn update(&self, problem: &OSQPSolverHandle) {
        if self.qp_params.q_mat.is_some() || self.qp_params.q_csc.is_some() {
            let q_csc = self.build_q_csc();
            problem.update_p(q_csc.clone());
        }

        if self.qp_params.q_vec.is_some() {
            let q_vec = self.qp_params.q_vec.clone().unwrap();
            problem.update_lin_cost(&q_vec);
        }

        if let Some(lb) = &self.qp_params.lb_vec {
            problem.update_lower_bound(lb.as_slice());
        }
        if let Some(ub) = &self.qp_params.ub_vec {
            problem.update_upper_bound(ub.as_slice());
        }

        if self.qp_params.a_mat.is_some() {
            let a = self.build_a_csc();
            problem.update_a(a);
        }
    }

    fn check(&self) -> Result<(), ModelError> {
        // If A is defined, then b must be defined. If b is defined, then A must be defined
        if self.qp_params.lb_vec.is_none()
            || self.qp_params.lb_vec.is_none()
            || self.qp_params.a_mat.is_none()
            || self.qp_params.ub_vec.is_none()
        {
            return Err(ModelError::ConfigError(
                "Incorrect configuration of constraints".into(),
            ));
        }

        if self.qp_params.q_mat.is_none() && self.qp_params.q_csc.is_none() {
            return Err(ModelError::ConfigError("q_mat is required".into()));
        }

        if self.qp_params.q_vec.is_none() {
            return Err(ModelError::ConfigError("q_vec is required".into()));
        }

        Ok(())
    }

    fn build_a_csc(&self) -> CscMatrix<'a> {
        let vec_mat = dmat_to_vec(&self.qp_params.a_mat.clone().unwrap());
        CscMatrix::from(&vec_mat)
    }

    fn build_q_csc(&self) -> CscMatrix<'a> {
        // Flatten q_mat into CSC format for OSQP
        if let Some(q_mat) = &self.qp_params.q_mat {
            CscMatrix::from(q_mat).into_upper_tri()
        } else {
            self.qp_params.q_csc.clone().unwrap()
        }
    }

    fn init(self) -> OSQPParams<'a> {
        let q_vec = self.qp_params.q_vec.clone().unwrap();

        // Flatten q_mat into CSC format for OSQP
        let q_csc = self.build_q_csc();
        // Stack A = [a_mat; g_mat]
        let a_csc = self.build_a_csc();
        // l and u
        let l = self.qp_params.lb_vec.unwrap().as_slice().to_vec();
        let u = self.qp_params.ub_vec.unwrap().as_slice().to_vec();
        let n_eq = l.iter().zip(u.iter()).filter(|(lb, ub)| lb == ub).count();
        let n_ineq = 2 * (l.len() - n_eq);

        (q_csc, q_vec, a_csc, l, u, n_eq, n_ineq)
    }
}
