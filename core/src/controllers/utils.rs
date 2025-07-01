use std::cmp::min;

use super::ConstraintTransform;
use crate::{
    controllers::ControllerOptions,
    physics::{
        ModelError,
        models::Dynamics,
        traits::{Discretizer, PhysicsSim, State},
    },
    utils::{Labelizable, evaluable::EvaluableMatrixFn},
};
use nalgebra::{DMatrix, DVector};

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

type LinearDynamicsEvaluation = (Vec<DMatrix<f64>>, Vec<DMatrix<f64>>);

pub(crate) fn linearize<S>(
    sim: &mut S,
    jacobian_x_fn: &EvaluableMatrixFn,
    jacobian_u_fn: &EvaluableMatrixFn,
    n_steps: usize,
    general_options: &ControllerOptions<S>,
) -> Result<LinearDynamicsEvaluation, ModelError>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model>,
{
    let n_op = min(general_options.get_u_operating().len(), n_steps - 1);
    let mut a_mat: Vec<DMatrix<f64>> = Vec::with_capacity(n_op);
    let mut b_mat: Vec<DMatrix<f64>> = Vec::with_capacity(n_op);
    let dt = general_options.get_dt();

    let mut real_params_opt: Option<Vec<f64>> = None;
    let labels = S::Model::labels();
    let real_params = sim.model().vectorize(labels);
    let estimated_params = if let Some(estimated_params) = general_options.get_estimated_params() {
        real_params_opt = Some(real_params);
        sim.update_model(estimated_params)?;
        estimated_params
    } else {
        real_params.as_slice()
    };

    for k in 0..n_op {
        let mut vals = general_options.concatenate_operating_point(k)?;
        vals.extend_from_slice(estimated_params);
        vals.extend_from_slice(&[dt]);
        a_mat.push(jacobian_x_fn.evaluate(&vals)?);
        b_mat.push(jacobian_u_fn.evaluate(&vals)?);
    }

    if let Some(real_params) = real_params_opt {
        sim.update_model(&real_params)?;
    }

    Ok((a_mat, b_mat))
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
