use super::common::RiccatiRecursionGeneric;
use super::options::RiccatiLQROptions;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};

pub struct RiccatiRecursionLQR<S: PhysicsSim>(RiccatiRecursionGeneric<S>);

impl<S> RiccatiRecursionLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x().clone());
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u().clone());

        let options = options.unwrap_or_default();

        let controller =
            RiccatiRecursionGeneric::new(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)?;

        Ok(RiccatiRecursionLQR(controller))
    }
}

impl<S> Controller<S> for RiccatiRecursionLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
