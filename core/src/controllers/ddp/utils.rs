use crate::{
    controllers::{ddp::DDPOptions, ControllerInput, ControllerOptions, CostFn},
    physics::traits::State,
};
use nalgebra::{DMatrix, DVector};
use crate::{
    controllers::ControllerState,
    physics::{ModelError, traits::PhysicsSim},
};


type CostExpansion = (DMatrix<f64>, DVector<f64>);




/// if terminal cost is Jn(x,u), return grad^2_x Jn(x,u), grad_x Jn(x,u)
pub(super) fn terminal_cost_expansion<S: PhysicsSim>(
    state: &[ControllerState<S>],
    cost_fn: &CostFn<S>,
) -> Result<CostExpansion, ModelError> {
    if state.is_empty() {
        return Err(ModelError::Unexpected(
            "State vector cannot be empty".into(),
        ));
    }
    let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
    let dstate_cost_dxx = cost_fn
        .get_qn()
        .unwrap_or(&DMatrix::<f64>::zeros(nx, nx))
        .to_owned();
    let terminal_cost_gradient = cost_fn.terminal_cost_gradient(state.last().unwrap());

    Ok((dstate_cost_dxx, terminal_cost_gradient))
}

/// if stage cost is J(x,u), return ((grad^2_x J(x,u), grad_x J(x,u)), (grad^2_u J(x,u), grad_u J(x,u)))
pub(super) fn stage_cost_expansion<S: PhysicsSim>(
    state: &[ControllerState<S>],
    input: &[ControllerInput<S>],
    stage: usize,
    cost_fn: &CostFn<S>,
) -> Result<(CostExpansion, CostExpansion), ModelError> {
    if state.is_empty() || input.is_empty() {
        return Err(ModelError::Unexpected(
            "State/Input vectors cannot be empty".into(),
        ));
    }
    let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
    let nu = ControllerInput::<S>::dim_q();
    let dstate_cost_dxx = cost_fn
        .get_q()
        .unwrap_or(&DMatrix::<f64>::zeros(nx, nx))
        .to_owned();
    let dinput_cost_duu = cost_fn
        .get_r()
        .unwrap_or(&DMatrix::<f64>::zeros(nu, nu))
        .to_owned();
    let (state_cost_gradient, input_cost_gradient) =
        cost_fn.stage_cost_gradient(&state[stage], &input[stage], stage)?;

    Ok((
        (dstate_cost_dxx, state_cost_gradient),
        (dinput_cost_duu, input_cost_gradient),
    ))
}

pub(super) fn update_operating_points<S: PhysicsSim>(
    x_traj: &[ControllerState<S>],
    u_traj: &[ControllerInput<S>],
    options: &DDPOptions<S>,
) -> ControllerOptions<S> {
    options
        .get_general()
        .to_owned()
        .set_u_operating(u_traj)
        .set_x_operating(x_traj)
}


pub (super) fn find_free_set(delta_u: &DVector<f64>, delta_lb: &DVector<f64>, delta_ub: &DVector<f64>) -> Vec<usize> {

    let tol = 1e-6;
    let mut free_indices = Vec::new();

    for i in 0..delta_u.len() {
        let near_lower = delta_u[i] <= delta_lb[i] + tol;
        let near_upper = delta_u[i] >=  delta_ub[i] + tol;
    
        if !(near_lower || near_upper) {
            free_indices.push(i);
        }
    }
    free_indices
}
