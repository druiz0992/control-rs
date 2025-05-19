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

    pub fn add_options(mut self, options: OptimizerConfig) -> Self {
        self.options = options;
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

        let q_len = self.q_vec.as_ref().unwrap().len();
        let b_len = b_vec.len();
        let mui = 0 + q_len..b_vec.len() + q_len;
        let sigmai = 0 + q_len + b_len..h_vec.len() + q_len + b_len;

        Ok(QP {
            xi: 0..q_len,
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{dmatrix, dvector};

    #[test]
    fn test_qp_builder_with_minimal_configuration() {
        let q_mat = dmatrix![1.0, 0.0; 0.0, 1.0];
        let q_vec = dvector![1.0, 1.0];

        let qp = QPBuilder::new()
            .q_mat(q_mat.clone())
            .q_vec(q_vec.clone())
            .build()
            .unwrap();

        assert_eq!(qp.q_mat, q_mat);
        assert_eq!(qp.q_vec, q_vec);
        assert_eq!(qp.a_mat, DMatrix::from_vec(1, 1, vec![0.0]));
        assert_eq!(qp.b_vec, DVector::from_row_slice(&[0.0]));
        assert_eq!(qp.g_mat, DMatrix::from_vec(1, 1, vec![0.0]));
        assert_eq!(qp.h_vec, DVector::from_row_slice(&[0.0]));
    }

    #[test]
    fn test_qp_builder_with_full_configuration() {
        let q_mat = dmatrix![1.0, 0.0; 0.0, 1.0];
        let q_vec = dvector![1.0, 1.0];
        let a_mat = dmatrix![1.0, 2.0];
        let b_vec = dvector![3.0];
        let g_mat = dmatrix![1.0, 0.0; 0.0, 1.0];
        let h_vec = dvector![4.0, 5.0];

        let qp = QPBuilder::new()
            .q_mat(q_mat.clone())
            .q_vec(q_vec.clone())
            .a_mat(a_mat.clone())
            .b_vec(b_vec.clone())
            .g_mat(g_mat.clone())
            .h_vec(h_vec.clone())
            .build()
            .unwrap();

        assert_eq!(qp.q_mat, q_mat);
        assert_eq!(qp.q_vec, q_vec);
        assert_eq!(qp.a_mat, a_mat);
        assert_eq!(qp.b_vec, b_vec);
        assert_eq!(qp.g_mat, g_mat);
        assert_eq!(qp.h_vec, h_vec);
    }

    #[test]
    fn test_qp_builder_missing_q_mat() {
        let q_vec = dvector![1.0, 1.0];

        let result = QPBuilder::new().q_vec(q_vec).build();

        assert!(result.is_err());
    }

    #[test]
    fn test_qp_builder_inconsistent_equality_constraints() {
        let a_mat = dmatrix![1.0, 2.0];

        let result = QPBuilder::new().a_mat(a_mat).build();

        assert!(result.is_err());
    }

    #[test]
    fn test_qp_builder_inconsistent_inequality_constraints() {
        let g_mat = dmatrix![1.0, 0.0; 0.0, 1.0];

        let result = QPBuilder::new().g_mat(g_mat).build();

        assert!(result.is_err());
    }
}
