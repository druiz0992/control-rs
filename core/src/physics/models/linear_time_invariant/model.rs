use crate::physics::ModelError;
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LtiModel<const N: usize, const C: usize, const I: usize> {
    state_matrix: DMatrix<f64>,
    control_matrix: DMatrix<f64>,
}

impl<const N: usize, const C: usize, const I: usize> LtiModel<N, C, I> {
    pub fn new(
        state_matrix: DMatrix<f64>,
        control_matrix: DMatrix<f64>,
    ) -> Result<Self, ModelError> {
        if state_matrix.nrows() != N || state_matrix.ncols() != N {
            return Err(ModelError::ConfigError(
                "LTI model requires square state matrix with {N} dimensions.".into(),
            ));
        }
        if control_matrix.nrows() != N || control_matrix.ncols() != I {
            return Err(ModelError::ConfigError(
                "LTI model requires control matrix with dimensions {N}x{C}.".into(),
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
