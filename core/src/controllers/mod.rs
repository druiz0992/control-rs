pub mod constraints;
pub mod indirect_shooting;
pub mod options;
pub mod qp_lqr;
pub mod qp_mpc;
pub mod riccati_lqr;
pub mod trajectory;
pub mod utils;

pub use constraints::ConstraintTransform;
pub use indirect_shooting::IndirectShooting;
use nalgebra::{DMatrix, DVector};
pub use options::ControllerOptions;
pub use qp_lqr::lqr::QPLQR;
pub use trajectory::InputTrajectory;

use crate::cost::CostFunction;
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{PhysicsSim, State};
use crate::utils::noise::NoiseSources;

type ControllerState<S> = <<S as PhysicsSim>::Model as Dynamics>::State;
type ControllerInput<S> = <<S as PhysicsSim>::Model as Dynamics>::Input;
type CostFn<S> = Box<dyn CostFunction<State = ControllerState<S>, Input = ControllerInput<S>>>;

pub type TrajectoryHistory<S> = (Vec<ControllerState<S>>, Vec<ControllerInput<S>>);
pub trait Controller<S: PhysicsSim> {
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError>;
}
pub trait UpdatableController<S: PhysicsSim>: Controller<S> {
    type Params<'a>;

    fn update_q(&self, state_ref: &[DVector<f64>], q: &mut DVector<f64>);
    fn update_a(
        &mut self,
        a_mat: &mut DMatrix<f64>,
        general_params: &ControllerOptions<S>,
    ) -> Result<(), ModelError>;
    fn update_bounds(&self, state: &DVector<f64>, lb: &mut DVector<f64>, ub: &mut DVector<f64>);
    fn update(&self, params: Self::Params<'_>);
}

pub trait SteppableController<S: PhysicsSim>: Controller<S> {
    fn step(
        &self,
        state: ControllerState<S>,
        input: Option<&ControllerInput<S>>,
        dt: f64,
    ) -> Result<ControllerState<S>, ModelError>;
}

fn input_from_slice<S: PhysicsSim>(slice: &[f64]) -> ControllerInput<S> {
    ControllerInput::<S>::from_slice(slice)
}
fn state_from_slice<S: PhysicsSim>(slice: &[f64]) -> ControllerState<S> {
    ControllerState::<S>::from_slice(slice)
}

fn into_clamped_input<S: PhysicsSim>(
    vector: DVector<f64>,
    u_limits: Option<&ConstraintTransform>,
) -> ControllerInput<S> {
    input_from_slice::<S>(utils::clamp_input_vector(vector, u_limits).as_slice())
}

fn try_into_noisy_state<S: PhysicsSim>(
    vector: DVector<f64>,
    noise_sources: &NoiseSources,
) -> Result<ControllerState<S>, ModelError> {
    let current_state = noise_sources
        .add_noise(vector)
        .map_err(ModelError::ConfigError)?;
    Ok(state_from_slice::<S>(current_state.as_slice()))
}
