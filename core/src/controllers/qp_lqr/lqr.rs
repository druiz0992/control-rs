use super::common::QPLQRGeneric;
use super::options::QPOptions;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};
use crate::solver::OSQPBuilder;
use crate::solver::osqp::builder::QPParams;

pub struct QPLQR<S: PhysicsSim>(QPLQRGeneric<S>);

impl<S> QPLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        x0: &ControllerState<S>,
        time_horizon: f64,
        dt: f64,
        options: Option<QPOptions<S>>,
    ) -> Result<(Self, QPParams), ModelError> {
        if time_horizon <= 0.0 || dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Incorrect time configuration".into(),
            ));
        }
        let jacobian_u = Box::new(sim.discretizer().jacobian_u().clone());
        let jacobian_x = Box::new(sim.discretizer().jacobian_x().clone());

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

        Ok((QPLQR(controller), updatable_qp_params))
    }
    pub fn update(&self, builder: OSQPBuilder) {
        self.0.update(builder);
    }
}

impl<S: PhysicsSim> Controller<S> for QPLQR<S> {
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
