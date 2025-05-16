use crate::physics::ModelError;
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LtiModel<const N: usize, const C: usize> {
    state_matrix: DMatrix<f64>,
    control_matrix: DMatrix<f64>,
}

impl<const N: usize, const C: usize> LtiModel<N, C> {
    pub fn new(
        state_matrix: DMatrix<f64>,
        control_matrix: DMatrix<f64>,
    ) -> Result<Self, ModelError> {
        if state_matrix.ncols() != state_matrix.nrows() {
            return Err(ModelError::ConfigError(
                "LTI model requires square state matrix.".into(),
            ));
        }
        if state_matrix.nrows() != control_matrix.nrows() {
            return Err(ModelError::ConfigError(
                "LTI model requires control matrix with same number of rows as state matrix."
                    .into(),
            ));
        }
        Ok(LtiModel {
            state_matrix,
            control_matrix,
        })
    }
    pub fn get_a_as_slice(&self) -> &DMatrix<f64> {
        &self.state_matrix
    }
    pub fn get_b_as_slice(&self) -> &DMatrix<f64> {
        &self.control_matrix
    }
}
