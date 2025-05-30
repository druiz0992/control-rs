use super::common::QPLQRGeneric;
use crate::controllers::options::ControllerOptions;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};
use crate::solver::OSQPBuilder;
use crate::solver::osqp::builder::QPParams;

pub struct QPLQRSymbolic<S: PhysicsSim>(QPLQRGeneric<S>);

impl<S> QPLQRSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        x0: &ControllerState<S>,
        time_horizon: f64,
        dt: f64,
        options: Option<ControllerOptions<S>>,
    ) -> Result<(Self, QPParams), ModelError> {
        let jacobian_u = Box::new(sim.discretizer().jacobian_u()?);
        let jacobian_x = Box::new(sim.discretizer().jacobian_x()?);

        let options = options.unwrap_or_default();

        let (controller, updatable_qp_params) = QPLQRGeneric::new(
            sim,
            cost_fn,
            jacobian_x,
            jacobian_u,
            x0,
            time_horizon,
            dt,
            options,
        )?;

        Ok((QPLQRSymbolic(controller), updatable_qp_params))
    }
    pub fn update(&self, builder: OSQPBuilder) {
        self.0.update(builder);
    }
}

impl<S: PhysicsSim> Controller<S> for QPLQRSymbolic<S> {
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
