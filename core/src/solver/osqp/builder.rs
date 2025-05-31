use super::solver::{OSQPSolver, OSQPSolverHandle};
use crate::physics::ModelError;
use core::f64;
use nalgebra::{DMatrix, DVector};
use osqp::{CscMatrix, Problem, Settings};

const INFINITY: f64 = 1e20;

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
    // equality constraints: a_mat * x - b_vec = 0
    pub a_mat: Option<DMatrix<f64>>,
    pub b_vec: Option<DVector<f64>>,
    // inequality constraints: g_mat * x - h_vec = 0
    pub g_mat: Option<DMatrix<f64>>,
    pub h_vec: Option<DVector<f64>>,
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

    pub fn b_vec(mut self, b_vec: DVector<f64>) -> Self {
        self.qp_params.b_vec = Some(b_vec);
        self
    }

    pub fn g_mat(mut self, g_mat: DMatrix<f64>) -> Self {
        self.qp_params.g_mat = Some(g_mat);
        self
    }

    pub fn h_vec(mut self, h_vec: DVector<f64>) -> Self {
        self.qp_params.h_vec = Some(h_vec);
        self
    }

    pub fn add_options(mut self, options: Settings) -> Self {
        self.options = options;
        self
    }

    pub fn build(self) -> Result<(OSQPSolverHandle, QPParams<'a>), ModelError> {
        self.check()?;
        let qp_params = self.qp_params.clone();
        let (q_csc, q, a_csc, l, u, n_eq, n_ineq) = self.init();
        let settings = Settings::default().verbose(false);

        let solver = OSQPSolver {
            problem: Problem::new(q_csc, &q, a_csc, &l, &u, &settings)
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

        if self.qp_params.b_vec.is_some() {
            let l = self.build_lower_bound();
            let u = self.build_upper_bound();
            problem.update_lower_bound(&l);
            problem.update_upper_bound(&u);
        }

        if self.qp_params.a_mat.is_some() {
            let a = self.build_a_csc();
            problem.update_a(a);
        }
    }

    fn check(&self) -> Result<(), ModelError> {
        if self.qp_params.b_vec.is_none() && self.qp_params.a_mat.is_some()
            || self.qp_params.b_vec.is_some() && self.qp_params.a_mat.is_none()
        {
            return Err(ModelError::ConfigError(
                "Incorrect configuration of equality constraints".into(),
            ));
        }
        if self.qp_params.h_vec.is_none() && self.qp_params.g_mat.is_some()
            || self.qp_params.h_vec.is_some() && self.qp_params.g_mat.is_none()
        {
            return Err(ModelError::ConfigError(
                "Incorrect configuration of inequality constraints".into(),
            ));
        }
        if self.qp_params.q_mat.is_none() && self.qp_params.q_csc.is_none() {
            return Err(ModelError::ConfigError("q_mat is required".into()));
        }

        if self.qp_params.b_vec.is_none() && self.qp_params.h_vec.is_none() {
            return Err(ModelError::ConfigError(
                "Some constraints are required".into(),
            ));
        }
        if self.qp_params.q_vec.is_none() {
            return Err(ModelError::ConfigError("q_vec is required".into()));
        }
        Ok(())
    }

    fn build_a_csc(&self) -> CscMatrix<'a> {
        // Stack A = [a_mat; g_mat]
        let a_combined = DMatrix::from_rows(
            &self
                .qp_params
                .a_mat
                .iter()
                .chain(self.qp_params.g_mat.iter())
                .flat_map(|m| m.row_iter())
                .collect::<Vec<_>>(),
        );

        let a_dense: Vec<Vec<_>> = (0..a_combined.nrows())
            .map(|i| a_combined.row(i).iter().cloned().collect())
            .collect();
        CscMatrix::from(&a_dense)
    }

    fn build_q_csc(&self) -> CscMatrix<'a> {
        // Flatten q_mat into CSC format for OSQP
        if let Some(q_mat) = &self.qp_params.q_mat {
            CscMatrix::from(q_mat).into_upper_tri()
        } else {
            self.qp_params.q_csc.clone().unwrap()
        }
    }

    fn build_lower_bound(&self) -> Vec<f64> {
        self.qp_params
            .b_vec
            .as_ref()
            .into_iter()
            .chain(self.qp_params.h_vec.as_ref())
            .flat_map(|v| v.iter().cloned())
            .collect()
    }

    fn build_upper_bound(&self) -> Vec<f64> {
        self.qp_params
            .b_vec
            .as_ref()
            .into_iter()
            .flat_map(|v| v.iter().cloned())
            .chain(
                self.qp_params
                    .h_vec
                    .as_ref()
                    .into_iter()
                    .flat_map(|v| std::iter::repeat(INFINITY).take(v.len())),
            )
            .collect()
    }

    fn init(&self) -> OSQPParams<'a> {
        let q_vec = self.qp_params.q_vec.clone().unwrap();
        let n_ineq = self.qp_params.h_vec.as_ref().map_or(0, |v| v.len());
        let n_eq = self.qp_params.b_vec.as_ref().map_or(0, |v| v.len());

        // Flatten q_mat into CSC format for OSQP
        let q_csc = self.build_q_csc();
        // Stack A = [a_mat; g_mat]
        let a_csc = self.build_a_csc();
        // l and u
        let l = self.build_lower_bound();
        let u = self.build_upper_bound();

        dbg!(&l, &u);

        (q_csc, q_vec, a_csc, l, u, n_eq, n_ineq)
    }
}
