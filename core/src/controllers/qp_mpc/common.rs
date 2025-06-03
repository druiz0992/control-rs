use crate::controllers::riccati_lqr::options::RiccatiLQROptions;
use crate::controllers::riccati_lqr::recursion::solve_steady_state_lqr;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, SteppableController, TrajectoryHistory,
    UpdatableController, state_from_slice, try_into_noisy_state,
};
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim, State};
use crate::solver::OSQPBuilder;
use crate::solver::osqp::builder::QPParams;
use crate::utils::evaluable::EvaluableDMatrix;
use crate::utils::noise::NoiseSources;
use nalgebra::{DMatrix, DVector};

use super::options::ConvexMpcOptions;

#[derive(Clone, Debug, Default)]
pub(super) struct ConvexMpcUpdatableParams {
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
pub(super) struct ConvexMpcGeneric<
    S: PhysicsSim,
    C: UpdatableController<S> + SteppableController<S>,
> {
    qp_controller: C,
    updatable_params: ConvexMpcUpdatableParams,

    n_steps: usize,
    state_matrix: DMatrix<f64>,
    running_cost: DMatrix<f64>,
    terminal_cost: DMatrix<f64>,

    options: ConvexMpcOptions<S>,
}

impl<S, C> ConvexMpcGeneric<S, C>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
    C: UpdatableController<S> + SteppableController<S>,
{
    pub(super) fn new(
        qp_controller: C,
        jacobian_x_fn: EvaluableDMatrix,
        jacobian_u_fn: EvaluableDMatrix,
        q_mat: DMatrix<f64>,
        r_mat: DMatrix<f64>,
        updatable_params: ConvexMpcUpdatableParams,
        options: ConvexMpcOptions<S>,
    ) -> Result<Self, ModelError> {
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        let vals = options.general.concatenate_operating_point();

        let a_mat = jacobian_x_fn.evaluate(&vals)?;
        let b_mat = jacobian_u_fn.evaluate(&vals)?;

        let ricatti_options = RiccatiLQROptions::enable_infinite_horizon();
        let (_, p_ss) =
            solve_steady_state_lqr::<S>(&a_mat, &b_mat, &q_mat, &r_mat, &ricatti_options)?;

        Ok(ConvexMpcGeneric {
            qp_controller,
            updatable_params,
            options,
            n_steps,
            state_matrix: a_mat,
            running_cost: q_mat,
            terminal_cost: p_ss,
        })
    }

    fn update_qp(&self, state: &DVector<f64>) -> OSQPBuilder<'_> {
        let ConvexMpcUpdatableParams {
            mut q_vec,
            mut lb_vec,
            mut ub_vec,
        } = self.updatable_params.clone();

        let n_steps = self.options.get_n_steps();

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

        dbg!(q_vec.len(), n_steps);
        for j in 0..n_steps - 1 {
            let offset = input_dims + j * (state_dims + input_dims);
            q_vec.rows_mut(offset, state_dims).copy_from(&q_x);
        }
        dbg!("XXX");

        // Final step (for j = Nh)
        let offset = input_dims + (n_steps - 1) * (state_dims + input_dims);
        dbg!("rrr");
        q_vec.rows_mut(offset, state_dims).copy_from(&qn_x);
        dbg!("ERERE");

        OSQPBuilder::new().q_vec(q_vec).bounds_vec(lb_vec, ub_vec)
    }
}

impl<S, C> Controller<S> for ConvexMpcGeneric<S, C>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
    C: UpdatableController<S> + SteppableController<S>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];
        let dt = self.options.get_general().get_dt();

        let (noise_0_std, noise_std) = self.options.general.get_noise().unwrap_or_default();

        let noise_sources = NoiseSources::from_stats(vec![(0.0, noise_0_std), (0.0, noise_std)])?;
        let mut current_state = state_from_slice::<S>(
            noise_sources
                .add_noise(0, x_traj[0].to_vector())?
                .as_mut_slice(),
        );

        // results are in r.0 : [u1, x2, u2, ...]
        for k in 0..self.n_steps - 1 {
            // 1- mpc_update
            // update controller
            self.update_qp(&x_traj[k].to_vector());
            let (_, mpc_u_traj) = self.qp_controller.solve(&current_state)?;
            u_traj[k] = mpc_u_traj[0].clone();

            current_state = self
                .qp_controller
                .step(current_state, Some(&u_traj[k]), dt)?;

            x_traj[k + 1] =
                try_into_noisy_state::<S>(current_state.to_vector(), &noise_sources, 1)?;
        }
        Ok((x_traj, u_traj))
    }
}
