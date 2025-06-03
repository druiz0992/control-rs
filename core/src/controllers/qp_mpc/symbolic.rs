use super::common::ConvexMpcGeneric;
use super::options::ConvexMpcOptions;
use crate::controllers::qp_lqr::options::QPOptions;
use crate::controllers::qp_lqr::symbolic::QPLQRSymbolic;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};

pub struct ConvexMpcSymbolic<S: PhysicsSim>(ConvexMpcGeneric<S, QPLQRSymbolic<S>>);

impl<S> ConvexMpcSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        state_0: &ControllerState<S>,
        options: Option<ConvexMpcOptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        if options.get_n_steps() >= n_steps {
            return Err(ModelError::ConfigError(format!(
                "MPC time steps {} must be less than overall number of time steps {}",
                options.get_n_steps(),
                n_steps
            )));
        }

        let jacobian_x = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u = Box::new(sim.discretizer().jacobian_u()?);

        let q_mat = cost_fn.get_q().cloned().ok_or(ModelError::ConfigError(
            "Expected non empty Q matrix".into(),
        ))?;

        let r_mat = cost_fn.get_r().cloned().ok_or(ModelError::ConfigError(
            "Expected non empty R matrix".into(),
        ))?;

        let qp_options = QPOptions::<S>::from(options.clone());

        let (qp_controller, updatable_qp_params) =
            QPLQRSymbolic::new(sim, cost_fn, state_0, Some(qp_options))?;

        let controller = ConvexMpcGeneric::new(
            qp_controller,
            jacobian_x,
            jacobian_u,
            q_mat,
            r_mat,
            updatable_qp_params.try_into()?,
            options,
        )?;

        Ok(ConvexMpcSymbolic(controller))
    }
}

impl<S> Controller<S> for ConvexMpcSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
