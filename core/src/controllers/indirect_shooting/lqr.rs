use super::IndirectShootingGeneric;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};

pub struct IndirectShootingLQR<S: PhysicsSim>(IndirectShootingGeneric<S>);

impl<S> IndirectShootingLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(sim: S, cost_fn: CostFn<S>, time_horizon: f64, dt: f64) -> Result<Self, ModelError> {
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u().clone());
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x().clone());

        let controller = IndirectShootingGeneric::<S>::new(
            sim,
            cost_fn,
            jacobian_x_fn,
            jacobian_u_fn,
            time_horizon,
            dt,
        )?;

        Ok(Self(controller))
    }
}

impl<S: PhysicsSim> Controller<S> for IndirectShootingLQR<S> {
    fn get_u_traj(&self) -> Vec<ControllerInput<S>> {
        self.0.get_u_traj()
    }

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        self.0.rollout(initial_state)
    }

    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError> {
        self.0.solve(initial_state)
    }
}
