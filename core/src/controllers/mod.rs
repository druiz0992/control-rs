pub mod indirect_shooting;
pub mod options;
pub mod qp_lqr;
pub mod qp_mpc;
pub mod riccati_lqr;
pub mod trajectory;
pub mod utils;

pub use indirect_shooting::lqr::IndirectShootingLQR;
pub use indirect_shooting::symbolic::IndirectShootingSymbolic;
use nalgebra::DVector;
pub use options::ControllerOptions;
pub use qp_lqr::lqr::QPLQR;
pub use riccati_lqr::lqr::RiccatiRecursionLQR;
pub use trajectory::InputTrajectory;

use crate::cost::CostFunction;
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{PhysicsSim, State};
use crate::utils::noise::NoiseSources;

type ControllerState<S> = <<S as PhysicsSim>::Model as Dynamics>::State;
type ControllerInput<S> = <<S as PhysicsSim>::Model as Dynamics>::Input;
type CostFn<S> = Box<dyn CostFunction<State = ControllerState<S>, Input = ControllerInput<S>>>;

pub trait Controller<S: PhysicsSim> {
    fn get_u_traj(&self) -> Vec<ControllerInput<S>>;

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError>;

    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError>;
}

fn input_from_slice<S: PhysicsSim>(slice: &[f64]) -> ControllerInput<S> {
    ControllerInput::<S>::from_slice(slice)
}
fn state_from_slice<S: PhysicsSim>(slice: &[f64]) -> ControllerState<S> {
    ControllerState::<S>::from_slice(slice)
}

fn into_clamped_input<S: PhysicsSim>(
    vector: DVector<f64>,
    u_limits: Option<(f64, f64)>,
) -> ControllerInput<S> {
    input_from_slice::<S>(utils::clamp_input_vector(vector, u_limits).as_slice())
}

fn try_into_noisy_state<S: PhysicsSim>(
    vector: DVector<f64>,
    noise_sources: &NoiseSources,
    idx: usize,
) -> Result<ControllerState<S>, ModelError> {
    let current_state = noise_sources.add_noise(idx, vector)?;
    Ok(state_from_slice::<S>(current_state.as_slice()))
}
