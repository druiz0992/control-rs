use super::common::RiccatiRecursionGeneric;
use super::options::RiccatiLQROptions;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};
use crate::utils::Labelizable;

pub struct RiccatiRecursionSymbolic<S: PhysicsSim>(RiccatiRecursionGeneric<S>);

impl<S> RiccatiRecursionSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics + Labelizable,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u()?);

        let options = options.unwrap_or_default();

        let controller =
            RiccatiRecursionGeneric::new(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)?;

        Ok(RiccatiRecursionSymbolic(controller))
    }
}

impl<S> Controller<S> for RiccatiRecursionSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics + Labelizable,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
