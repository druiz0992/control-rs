use std::env::consts::OS;

use nalgebra::{DMatrix, DVector};

use crate::controllers::options::ControllerOptions;
use crate::controllers::qp_lqr::options::QPOptions;
use crate::controllers::qp_lqr::symbolic::QPLQRSymbolic;
use crate::controllers::riccati_lqr::options::RiccatiLQROptions;
use crate::controllers::riccati_lqr::recursion::solve_steady_state_lqr;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, CostFn, QPLQR, RiccatiRecursionLQR,
    TrajectoryHistory,
};
use crate::physics::ModelError;
use crate::physics::discretizer::{LinearDiscretizer, SymbolicDiscretizer};
use crate::physics::traits::{LinearDynamics, PhysicsSim, State, SymbolicDynamics};
use crate::solver::osqp::builder::{self, QPParams};
use crate::solver::{OSQPBuilder, QPBuilder};
use crate::utils::evaluable::Evaluable;

use super::options::ConvexMpcOptions;

#[derive(Clone, Debug, Default)]
struct ConvexMpcUpdatableParams {
    q_vec: DVector<f64>,
    lb_vec: DVector<f64>,
    ub_vec: DVector<f64>,
}

impl<'a> TryFrom<QPParams<'a>> for ConvexMpcUpdatableParams {
    type Error = ModelError;
    fn try_from(value: QPParams<'a>) -> Result<Self, Self::Error> {
        let q_vec = value
            .q_vec
            .ok_or(ModelError::Other("Missing q_vec".into()))?;
        let lb_vec = value
            .lb_vec
            .ok_or(ModelError::Other("Missing Lower bound vector".into()))?;
        let ub_vec = value
            .ub_vec
            .ok_or(ModelError::Other("Missing Upper bound vector".into()))?;
        Ok(ConvexMpcUpdatableParams {
            q_vec: DVector::from_vec(q_vec),
            lb_vec,
            ub_vec,
        })
    }
}
pub struct ConvexMpc<S: PhysicsSim> {
    qp_controller: QPLQRSymbolic<S>,
    updatable_params: ConvexMpcUpdatableParams,

    n_steps: usize,
    state_matrix: DMatrix<f64>,
    running_cost: DMatrix<f64>,
    terminal_cost: DMatrix<f64>,

    options: ConvexMpcOptions<S>,
}

impl<S> ConvexMpc<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        state_0: &ControllerState<S>,
        time_horizon: f64,
        dt: f64,
        options: Option<ConvexMpcOptions<S>>,
    ) -> Result<Self, ModelError> {
        let n_steps = (time_horizon / dt) as usize + 1;
        let options = options.unwrap_or_default();

        if options.get_n_steps() >= n_steps {
            return Err(ModelError::ConfigError(format!(
                "MPC time steps {} must be less than overall number of time steps {}",
                options.get_n_steps(),
                n_steps
            )));
        }

        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u()?);

        let vals = options.general.concatenate_operating_point();

        let a_mat = jacobian_x_fn.evaluate(&vals)?;
        let b_mat = jacobian_u_fn.evaluate(&vals)?;
        let q_mat = cost_fn.get_q().cloned().ok_or(ModelError::ConfigError(
            "Expected non empty Q matrix".into(),
        ))?;

        let r_mat = cost_fn.get_r().ok_or(ModelError::ConfigError(
            "Expected non empty R matrix".into(),
        ))?;

        let ricatti_options = RiccatiLQROptions::enable_infinite_horizon();
        let (_, p_ss) =
            solve_steady_state_lqr::<S>(&a_mat, &b_mat, &q_mat, r_mat, &ricatti_options)?;

        let (qp_controller, updatable_qp_params) = QPLQRSymbolic::new(
            sim,
            cost_fn,
            state_0,
            options.get_n_steps() as f64 * dt,
            dt,
            Some(QPOptions::<S>::from(options.clone())),
        )?;
        Ok(ConvexMpc {
            qp_controller,
            updatable_params: updatable_qp_params.try_into()?,
            options,
            n_steps,
            state_matrix: a_mat,
            running_cost: q_mat,
            terminal_cost: p_ss,
        })
    }

    fn update_qp(&self, state: &DVector<f64>) {
        let ConvexMpcUpdatableParams {
            mut q_vec,
            mut lb_vec,
            mut ub_vec,
        } = self.updatable_params.clone();
        // update vector d[0] to -A * x0; => lb <= Az <= ub;
        let a_x = -&self.state_matrix * state;
        lb_vec.rows_mut(0, state.len()).copy_from(&a_x);
        ub_vec.rows_mut(0, state.len()).copy_from(&a_x);

        // update vector b with
        let state_dims = state.len();
        let input_dims = ControllerInput::<S>::dim_q();
        // TODO >>> review
        let state_ref = self.options.general.get_x_ref()[0].to_vector();
        let q_x = -&self.running_cost * &state_ref;
        let qn_x = -&self.terminal_cost * &state_ref;

        for j in 0..self.n_steps - 1 {
            let offset = input_dims + (j - 1) * (state_dims + input_dims);
            q_vec.rows_mut(offset, state_dims).copy_from(&q_x);
        }

        // Final step (for j = Nh)
        let offset = input_dims + (self.n_steps - 1) * (state_dims + input_dims);
        q_vec.rows_mut(offset, state_dims).copy_from(&qn_x);

        let builder = OSQPBuilder::new().q_vec(q_vec).bounds_vec(lb_vec, ub_vec);
        self.qp_controller.update(builder);
    }

    pub fn update_controller(&self) {}
}

impl<S: PhysicsSim> Controller<S> for ConvexMpc<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        // init
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];

        // retuls are in r.0 : [u1, x2, u2, ...]

        // 1- mpc_update
        // update controller

        // solve problem
        //let u_ref = self.options.get_u_equilibrium();
        let delta_u = self.qp_controller.solve(initial_state)?.1[0].clone();
        //let updated_u = delta_u.to_vector() + u_ref.to_vector();

        // 2 - closed loop controller
        todo!();
    }
}
