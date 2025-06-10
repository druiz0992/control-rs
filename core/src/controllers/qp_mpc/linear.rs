use super::common::ConvexMpcGeneric;
use super::options::ConvexMpcOptions;
use crate::controllers::qp_lqr::options::QPOptions;
use crate::controllers::riccati_lqr::options::RiccatiLQROptions;
use crate::controllers::riccati_lqr::recursion::solve_steady_state_lqr;
use crate::controllers::{
    Controller, ControllerState, CostFn, QPLQR, SteppableController, TrajectoryHistory,
    UpdatableController,
};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};
use crate::utils::evaluable::Evaluable;

pub struct ConvexMpc<S: PhysicsSim, C: SteppableController<S> + UpdatableController<S>>(
    ConvexMpcGeneric<S, C>,
);

impl<S> ConvexMpc<S, QPLQR<S>>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        mut cost_fn: CostFn<S>,
        state_0: &ControllerState<S>,
        options: Option<ConvexMpcOptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        // check qp horizon is shorter than full horizon
        let finitite_horizon = options.get_horizon();
        if finitite_horizon >= options.get_general().get_time_horizon() {
            return Err(ModelError::ConfigError(format!(
                "MPC horizon {} must be less than overall horizon {}",
                options.get_horizon(),
                options.get_general().get_time_horizon()
            )));
        }

        let jacobian_x = Box::new(sim.discretizer().jacobian_x().clone());
        let jacobian_u = Box::new(sim.discretizer().jacobian_u().clone());

        let q_mat = cost_fn.get_q().cloned().ok_or(ModelError::ConfigError(
            "Expected non empty Q matrix".into(),
        ))?;

        let r_mat = cost_fn.get_r().cloned().ok_or(ModelError::ConfigError(
            "Expected non empty R matrix".into(),
        ))?;

        let vals = options.general.concatenate_operating_point(0)?;
        let a_mat = jacobian_x.evaluate(&vals)?;
        let b_mat = jacobian_u.evaluate(&vals)?;

        let ricatti_options = RiccatiLQROptions::enable_infinite_horizon();
        let (p_ss, _) =
            solve_steady_state_lqr::<S>(&a_mat, &b_mat, &q_mat, &r_mat, &ricatti_options)?;

        // update qp horizon with mpc horizon
        let general_options = options
            .get_general()
            .clone()
            .set_time_horizon(finitite_horizon)?;
        let qp_options = QPOptions::<S>::from(options.clone()).set_general(general_options);

        cost_fn.update_qn(p_ss.clone())?;

        let (qp_controller, updatable_qp_params) =
            QPLQR::new(sim, cost_fn, state_0, Some(qp_options))?;

        let controller =
            ConvexMpcGeneric::new(qp_controller, updatable_qp_params.try_into()?, options)?;

        Ok(ConvexMpc(controller))
    }
}

impl<S> Controller<S> for ConvexMpc<S, QPLQR<S>>
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
