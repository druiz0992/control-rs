use super::common::RiccatiRecursionGeneric;
use super::options::RiccatiLQROptions;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};

pub struct RiccatiRecursionSymbolic<S: PhysicsSim>(RiccatiRecursionGeneric<S>);

impl<S> RiccatiRecursionSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        time_horizon: f64,
        dt: f64,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u()?);

        let options = options.unwrap_or_default();

        let controller = RiccatiRecursionGeneric::new(
            sim,
            cost_fn,
            jacobian_x_fn,
            jacobian_u_fn,
            time_horizon,
            dt,
            options,
        )?;

        Ok(RiccatiRecursionSymbolic(controller))
    }
}

impl<S> Controller<S> for RiccatiRecursionSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
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
