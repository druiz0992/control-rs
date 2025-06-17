use nalgebra::DMatrix;

use super::common::RiccatiRecursionGeneric;
use super::options::RiccatiLQROptions;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim, SymbolicDynamics};
use crate::utils::Labelizable;
use crate::utils::evaluable::Evaluable;

pub struct RiccatiRecursionNumeric<S: PhysicsSim>(RiccatiRecursionGeneric<S>);

impl<S> RiccatiRecursionNumeric<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model> + Evaluable,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        let evaluable_f = sim.discretizer();
        let controller = RiccatiRecursionGeneric::new(
            sim,
            cost_fn,
            Box::new(jacobian_fn),
            jacobian_u_fn,
            options,
        )?;

        Ok(RiccatiRecursionNumeric(controller))
    }
}

impl<S> Controller<S> for RiccatiRecursionNumeric<S>
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
