use super::common::IndirectShootingGeneric;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
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
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
