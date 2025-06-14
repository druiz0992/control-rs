
use crate::physics::ModelError;
use crate::physics::traits::{PhysicsSim, State};
use nalgebra::DMatrix;
use super::ControllerInput;

/// Represents a trajectory of controller inputs for a physics simulation.
///
/// This struct encapsulates a sequence of `ControllerInput` instances, which can be used
/// to define a trajectory for controlling a physics simulation.
///
/// # Type Parameters
/// - `S`: A type that implements the `PhysicsSim` trait, representing the physics simulation.
///
/// # Errors
/// - `ModelError::ConfigError`: Raised when attempting to convert an empty matrix into an `InputTrajectory`.
pub struct InputTrajectory<S: PhysicsSim>(Vec<ControllerInput<S>>);

impl<S: PhysicsSim> InputTrajectory<S> {
    pub fn new(input: Vec<ControllerInput<S>>) -> Self {
        Self(input)
    }

    pub fn as_slice(&self) -> &[ControllerInput<S>] {
        &self.0
    }
    pub fn as_vec(&self) -> &Vec<ControllerInput<S>> {
        &self.0
    }
    pub fn as_mut_vec(&mut self) -> &Vec<ControllerInput<S>> {
        &self.0
    }
    pub fn to_vec(&self) -> Vec<ControllerInput<S>> {
        self.0.clone()
    }
}

impl<S: PhysicsSim> From<&InputTrajectory<S>> for DMatrix<f64> {
    fn from(value: &InputTrajectory<S>) -> DMatrix<f64> {
        if value.0.is_empty() {
            return DMatrix::from_column_slice(0, 0, &[]);
        }
        let input_dims = value.0[0].to_vec().len();
        let mut mat = DMatrix::<f64>::zeros(input_dims, value.0.len());
        value.0.iter().enumerate().for_each(|(i, v)| {
            mat.set_column(i, &v.to_vector());
        });
        mat
    }
}

impl<S: PhysicsSim> TryFrom<&DMatrix<f64>> for InputTrajectory<S> {
    type Error = ModelError;
    fn try_from(value: &DMatrix<f64>) -> Result<Self, Self::Error> {
        if value.is_empty() {
            return Err(ModelError::ConfigError(
                "Input matrix cannot be empty".into(),
            ));
        }

        Ok(InputTrajectory(
            value
                .column_iter()
                .map(|v| ControllerInput::<S>::from_slice(v.as_slice()))
                .collect(),
        ))
    }
}
