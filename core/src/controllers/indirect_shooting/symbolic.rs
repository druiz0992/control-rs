use super::common::IndirectShootingGeneric;
use crate::controllers::{
    Controller, ControllerOptions, ControllerState, CostFn, TrajectoryHistory,
};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};

pub struct IndirectShootingSymbolic<S: PhysicsSim>(IndirectShootingGeneric<S>);

impl<S> IndirectShootingSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<ControllerOptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u()?);

        let controller =
            IndirectShootingGeneric::<S>::new(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)?;

        Ok(Self(controller))
    }
}

impl<S: PhysicsSim> Controller<S> for IndirectShootingSymbolic<S> {
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
