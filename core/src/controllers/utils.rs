use super::ConstraintTransform;
use crate::physics::traits::State;
use nalgebra::DVector;

/// Clamps an input vector within given limits
pub fn clamp_input_vector(
    input: DVector<f64>,
    limits: Option<&ConstraintTransform>,
) -> DVector<f64> {
    if let Some(constraint) = limits {
        let (lower, upper) = constraint.bounds_as_slice();
        DVector::from_iterator(
            input.len(),
            input
                .iter()
                .zip(lower.iter().zip(upper.iter()))
                .map(|(xi, (lo, hi))| xi.clamp(*lo, *hi)),
        )
    } else {
        input
    }
}
pub fn extend_vector<T: State>(vec: &[T], start: usize, end: usize) -> Vec<DVector<f64>> {
    let mut vec_ref: Vec<_> = vec
        .iter()
        .skip(start)
        .take(end - start)
        .map(|x_ref| x_ref.to_vector())
        .collect();

    let missing = (end - start).saturating_sub(vec_ref.len());
    let last_ref = vec.last().unwrap().to_vector();
    vec_ref.extend(std::iter::repeat(last_ref).take(missing));
    vec_ref
}

/*
/// Adds gaussian noise to controller input and returns the updated input sample
pub fn add_noise_to_inputs<S: PhysicsSim>(
    std_dev: f64,
    inputs: Vec<ControllerInput<S>>,
) -> Result<Vec<ControllerInput<S>>, ModelError> {
    let noise_source = NoiseSource::new(std_dev)?;

    Ok(inputs
        .iter()
        .map(|s| ControllerInput::<S>::from_slice(noise_source.add_noise(s.to_vector()).as_slice()))
        .collect())
}

/// Adds gaussian noise to controller state and returns the updated state sample
pub fn add_noise_to_states<S: PhysicsSim>(
    std_dev: f64,
    states: Vec<ControllerState<S>>,
) -> Result<Vec<ControllerState<S>>, ModelError> {
    let noise_source = NoiseSource::new(std_dev)?;

    Ok(states
        .iter()
        .map(|s| ControllerState::<S>::from_slice(noise_source.add_noise(s.to_vector()).as_slice()))
        .collect())
}
        */
