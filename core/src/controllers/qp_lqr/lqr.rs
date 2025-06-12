use nalgebra::DVector;

use super::common::QPLQRGeneric;
use super::options::QPOptions;
use crate::controllers::{
    Controller, ControllerInput, ControllerOptions, ControllerState, CostFn, SteppableController,
    TrajectoryHistory, UpdatableController,
};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};
use crate::solver::OSQPBuilder;
use crate::solver::osqp::builder::QPParams;
use crate::utils::Labelizable;

pub struct QPLQR<S: PhysicsSim>(QPLQRGeneric<S>);

impl<S> QPLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics + Labelizable,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        x0: &ControllerState<S>,
        options: Option<QPOptions<S>>,
    ) -> Result<(Self, QPParams), ModelError> {
        let options = options.unwrap_or_default();
        let jacobian_u = Box::new(sim.discretizer().jacobian_u().clone());
        let jacobian_x = Box::new(sim.discretizer().jacobian_x().clone());

        let (controller, updatable_qp_params) =
            QPLQRGeneric::new(sim, cost_fn, jacobian_x, jacobian_u, x0, options)?;

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

impl<S: PhysicsSim> SteppableController<S> for QPLQR<S> {
    fn step(
        &self,
        state: ControllerState<S>,
        input: Option<&ControllerInput<S>>,
        dt: f64,
    ) -> Result<ControllerState<S>, ModelError> {
        self.0.step(state, input, dt)
    }
}

impl<S: PhysicsSim> UpdatableController<S> for QPLQR<S>
where
    S::Model: LinearDynamics + Labelizable,
{
    type Params<'a> = OSQPBuilder<'a>;

    fn update(&self, params: Self::Params<'_>) {
        self.0.update(params);
    }
    fn update_bounds(&self, state: &DVector<f64>, lb: &mut DVector<f64>, ub: &mut DVector<f64>) {
        self.0.update_bounds(state, lb, ub);
    }
    fn update_q(&self, state_ref: &[DVector<f64>], q: &mut DVector<f64>) {
        self.0.update_q(state_ref, q);
    }
    fn update_a(
        &mut self,
        a_mat: &mut nalgebra::DMatrix<f64>,
        general_params: &ControllerOptions<S>,
    ) -> Result<(), ModelError> {
        self.0.update_a(a_mat, general_params)
    }
}
