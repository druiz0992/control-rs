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
type LinearDynamicsSingleEvaluation = (DMatrix<f64>, DMatrix<f64>);
type LinearDynamicsSecondOrderSingleEvaluation =
    (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>, DMatrix<f64>);

pub(crate) fn linearize<S>(
    sim: &S,
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

    let labels = S::Model::labels();
    let real_params = sim.model().vectorize(labels);
    let model_params = if let Some(estimated_params) = general_options.get_estimated_params() {
        estimated_params
    } else {
        real_params.as_slice()
    };

    for k in 0..n_op {
        let mut vals = general_options.concatenate_operating_point(k)?;
        vals.extend_from_slice(model_params);
        vals.extend_from_slice(&[dt]);
        a_mat.push(jacobian_x_fn.evaluate(&vals)?);
        b_mat.push(jacobian_u_fn.evaluate(&vals)?);
    }

    Ok((a_mat, b_mat))
}

pub(crate) fn linearize_one<S>(
    sim: &S,
    jacobian_x_fn: &EvaluableMatrixFn,
    jacobian_u_fn: &EvaluableMatrixFn,
    idx: usize,
    general_options: &ControllerOptions<S>,
) -> Result<LinearDynamicsSingleEvaluation, ModelError>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model>,
{
    let dt = general_options.get_dt();
    let labels = S::Model::labels();
    let real_params = sim.model().vectorize(labels);
    let model_params = if let Some(estimated_params) = general_options.get_estimated_params() {
        estimated_params
    } else {
        real_params.as_slice()
    };

    let mut vals = general_options.concatenate_operating_point(idx)?;
    vals.extend_from_slice(model_params);
    vals.extend_from_slice(&[dt]);
    let a_mat = jacobian_x_fn.evaluate(&vals)?;
    let b_mat = jacobian_u_fn.evaluate(&vals)?;

    Ok((a_mat, b_mat))
}

pub(crate) fn linearize_second_order_one<S>(
    sim: &S,
    hessian_xx_fn: &EvaluableMatrixFn,
    hessian_xu_fn: &EvaluableMatrixFn,
    hessian_ux_fn: &EvaluableMatrixFn,
    hessian_uu_fn: &EvaluableMatrixFn,
    idx: usize,
    general_options: &ControllerOptions<S>,
) -> Result<LinearDynamicsSecondOrderSingleEvaluation, ModelError>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model>,
{
    let dt = general_options.get_dt();
    let labels = S::Model::labels();
    let real_params = sim.model().vectorize(labels);
    let model_params = if let Some(estimated_params) = general_options.get_estimated_params() {
        estimated_params
    } else {
        real_params.as_slice()
    };

    let mut vals = general_options.concatenate_operating_point(idx)?;
    vals.extend_from_slice(model_params);
    vals.extend_from_slice(&[dt]);
    let a_x = hessian_xx_fn.evaluate(&vals)?;
    let a_u = hessian_xu_fn.evaluate(&vals)?;
    let b_x = hessian_ux_fn.evaluate(&vals)?;
    let b_u = hessian_uu_fn.evaluate(&vals)?;

    Ok((a_x, a_u, b_x, b_u))
}
