use nalgebra::DVector;

use crate::physics::ModelError;
use crate::physics::traits::{PhysicsSim, State};
use crate::utils::NoiseSource;

use super::{ControllerInput, ControllerState};

pub fn clamp_input_vector(input: DVector<f64>, limits: Option<(f64, f64)>) -> DVector<f64> {
    if let Some((lower, upper)) = limits {
        input.map(|xi| xi.clamp(lower, upper))
    } else {
        input
    }
}

pub fn add_noise_to_inputs<S: PhysicsSim>(
    mean: f64,
    std_dev: f64,
    inputs: Vec<ControllerInput<S>>,
) -> Result<Vec<ControllerInput<S>>, ModelError> {
    let noise_source = NoiseSource::new(mean, std_dev)?;

    Ok(inputs
        .iter()
        .map(|s| {
            ControllerInput::<S>::from_slice(&noise_source.add_noise(s.to_vector()).as_slice())
        })
        .collect())
}

pub fn add_noise_to_states<S: PhysicsSim>(
    mean: f64,
    std_dev: f64,
    states: Vec<ControllerState<S>>,
) -> Result<Vec<ControllerState<S>>, ModelError> {
    let noise_source = NoiseSource::new(mean, std_dev)?;

    Ok(states
        .iter()
        .map(|s| {
            ControllerState::<S>::from_slice(&noise_source.add_noise(s.to_vector()).as_slice())
        })
        .collect())
}
