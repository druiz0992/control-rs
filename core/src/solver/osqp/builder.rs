use super::solver::{OSQPSolver, OSQPSolverHandle};
use crate::physics::ModelError;
use core::f64;
use nalgebra::{DMatrix, DVector};
use osqp::{CscMatrix, Problem, Settings};

const INFINITY: f64 = 1e20;

#[derive(Clone, Default)]
pub struct OSQPBuilder<'a> {
    // objective function : 0.5 * x' * q_mat * x + q_vec' * x
    q_csc: Option<CscMatrix<'a>>,
    q_mat: Option<Vec<Vec<f64>>>,
    q_vec: Option<Vec<f64>>,
    // equality constraints: a_mat * x - b_vec = 0
    a_mat: Option<DMatrix<f64>>,
    b_vec: Option<DVector<f64>>,
    // inequality constraints: g_mat * x - h_vec = 0
    g_mat: Option<DMatrix<f64>>,
    h_vec: Option<DVector<f64>>,

    options: Settings,
}

impl<'a> OSQPBuilder<'a> {
    pub fn new() -> Self {
        Self {
            q_csc: None,
            q_mat: None,
            q_vec: None,
            a_mat: None,
            b_vec: None,
            g_mat: None,
            h_vec: None,

            options: Settings::default(),
        }
    }

    pub fn q_mat(mut self, q_mat: DMatrix<f64>) -> Self {
        let q = (0..q_mat.nrows())
            .map(|i| q_mat.row(i).iter().cloned().collect())
            .collect();
        self.q_mat = Some(q);
        self
    }
    pub fn q_csc(mut self, q_csc: CscMatrix<'a>) -> Self {
        self.q_csc = Some(q_csc);
        self
    }

    pub fn q_vec(mut self, q_vec: DVector<f64>) -> Self {
        self.q_vec = Some(q_vec.as_slice().to_vec());
        self
    }

    pub fn a_mat(mut self, a_mat: DMatrix<f64>) -> Self {
        self.a_mat = Some(a_mat);
        self
    }

    pub fn b_vec(mut self, b_vec: DVector<f64>) -> Self {
        self.b_vec = Some(b_vec);
        self
    }

    pub fn g_mat(mut self, g_mat: DMatrix<f64>) -> Self {
        self.g_mat = Some(g_mat);
        self
    }

    pub fn h_vec(mut self, h_vec: DVector<f64>) -> Self {
        self.h_vec = Some(h_vec);
        self
    }

    pub fn add_options(mut self, options: Settings) -> Self {
        self.options = options;
        self
    }

    pub fn build(self) -> Result<OSQPSolverHandle, ModelError> {
        if self.b_vec.is_none() && self.a_mat.is_some()
            || self.b_vec.is_some() && self.a_mat.is_none()
        {
            return Err(ModelError::ConfigError(
                "Incorrect configuration of equality constraints".into(),
            ));
        }
        if self.h_vec.is_none() && self.g_mat.is_some()
            || self.h_vec.is_some() && self.g_mat.is_none()
        {
            return Err(ModelError::ConfigError(
                "Incorrect configuration of inequality constraints".into(),
            ));
        }
        if self.q_mat.is_none() && self.q_csc.is_none() {
            return Err(ModelError::ConfigError("q_mat is required".into()));
        }

        if self.b_vec.is_none() && self.h_vec.is_none() {
            return Err(ModelError::ConfigError(
                "Some constraints are required".into(),
            ));
        }

        let q_vec = self
            .q_vec
            .ok_or(ModelError::ConfigError("q_vec is required".into()))?;

        let n_ineq = self.h_vec.as_ref().map_or(0, |v| v.len());
        let n_eq = self.b_vec.as_ref().map_or(0, |v| v.len());

        // Flatten q_mat into CSC format for OSQP
        let q_csc;
        if let Some(q_mat) = self.q_mat {
            q_csc = CscMatrix::from(&q_mat).into_upper_tri();
        } else {
            q_csc = self.q_csc.unwrap();
        }
        // q vector
        let q = q_vec;

        // Stack A = [a_mat; g_mat]
        let a_combined = DMatrix::from_rows(
            &self
                .a_mat
                .iter()
                .chain(self.g_mat.iter())
                .flat_map(|m| m.row_iter())
                .collect::<Vec<_>>(),
        );

        let a_dense: Vec<Vec<_>> = (0..a_combined.nrows())
            .map(|i| a_combined.row(i).iter().cloned().collect())
            .collect();
        let a_csc = CscMatrix::from(&a_dense);

        // l and u
        let l: Vec<f64> = self
            .b_vec
            .as_ref()
            .into_iter()
            .chain(self.h_vec.as_ref())
            .flat_map(|v| v.iter().cloned())
            .collect();

        let u: Vec<f64> = self
            .b_vec
            .as_ref()
            .into_iter()
            .flat_map(|v| v.iter().cloned())
            .chain(
                self.h_vec
                    .as_ref()
                    .into_iter()
                    .flat_map(|v| std::iter::repeat(INFINITY).take(v.len())),
            )
            .collect();

        let settings = Settings::default().verbose(false);

        let solver = OSQPSolver {
            problem: Problem::new(q_csc, &q, a_csc, &l, &u, &settings)
                .map_err(|e| ModelError::ConfigError(e.to_string()))?,
            n_eq,
            n_ineq,
        };

        Ok(OSQPSolverHandle::new(solver))
    }
}
