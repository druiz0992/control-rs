use super::QP;
use crate::{numeric_services::solver::OptimizerConfig, physics::ModelError};
use nalgebra::{DMatrix, DVector};

#[derive(Clone, Default)]
pub struct QPBuilder {
    q_mat: Option<DMatrix<f64>>,
    q_vec: Option<DVector<f64>>,
    a_mat: Option<DMatrix<f64>>,
    b_vec: Option<DVector<f64>>,
    g_mat: Option<DMatrix<f64>>,
    h_vec: Option<DVector<f64>>,

    options: OptimizerConfig,
}

impl QPBuilder {
    pub fn new() -> Self {
        Self {
            q_mat: None,
            q_vec: None,
            a_mat: None,
            b_vec: None,
            g_mat: None,
            h_vec: None,

            options: OptimizerConfig::default(),
        }
    }

    pub fn q_mat(mut self, q_mat: DMatrix<f64>) -> Self {
        self.q_mat = Some(q_mat);
        self
    }

    pub fn q_vec(mut self, q_vec: DVector<f64>) -> Self {
        self.q_vec = Some(q_vec);
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

    pub fn build(self) -> Result<QP, ModelError> {
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

        let b_vec = self.b_vec.unwrap_or(DVector::from_row_slice(&[0.0]));
        let h_vec = self.h_vec.unwrap_or(DVector::from_row_slice(&[0.0]));
        let a_mat = self.a_mat.unwrap_or(DMatrix::from_vec(1, 1, vec![0.0]));
        let g_mat = self.g_mat.unwrap_or(DMatrix::from_vec(1, 1, vec![0.0]));

        let mui = 0..b_vec.len();
        let sigmai = 0..h_vec.len();

        Ok(QP {
            xi: 0..self.q_vec.as_ref().unwrap().len(),
            mui,
            sigmai,
            q_mat: self
                .q_mat
                .ok_or(ModelError::ConfigError("q_mat is required".into()))?,
            q_vec: self
                .q_vec
                .ok_or(ModelError::ConfigError("q_vec is required".into()))?,
            a_mat,
            b_vec,
            g_mat,
            h_vec,
            options: self.options,
            status: None,
        })
    }
}
