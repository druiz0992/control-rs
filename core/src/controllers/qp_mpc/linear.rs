use super::common::ConvexMpcGeneric;
use super::options::ConvexMpcOptions;
use crate::controllers::qp_lqr::options::QPOptions;
use crate::controllers::{
    Controller, ControllerState, CostFn, QPLQR, SteppableController, TrajectoryHistory,
    UpdatableController,
};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim};

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
        cost_fn: CostFn<S>,
        state_0: &ControllerState<S>,
        options: Option<ConvexMpcOptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();
        // full horizon
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        // check qp horizon is shorted than full horizon
        let finitite_horizon = options.get_n_steps();
        if finitite_horizon >= n_steps {
            return Err(ModelError::ConfigError(format!(
                "MPC time steps {} must be less than overall number of time steps {}",
                options.get_n_steps(),
                n_steps
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

        let general_options = options.get_general().clone().set_time_horizon(0.95)?;
        let qp_options = QPOptions::<S>::from(options.clone()).set_general(general_options);

        let (qp_controller, updatable_qp_params) =
            QPLQR::new(sim, cost_fn, state_0, Some(qp_options))?;

        let controller = ConvexMpcGeneric::new(
            qp_controller,
            jacobian_x,
            jacobian_u,
            q_mat,
            r_mat,
            updatable_qp_params.try_into()?,
            options,
        )?;

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
