use crate::controllers::utils::extend_vector;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, SteppableController, TrajectoryHistory,
    UpdatableController, state_from_slice, try_into_noisy_state,
};
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim, State};
use crate::solver::OSQPBuilder;
use crate::solver::osqp::builder::QPParams;
use crate::utils::noise::NoiseSources;
use nalgebra::{DMatrix, DVector};

use super::options::ConvexMpcOptions;

#[derive(Clone, Debug, Default)]
pub(super) struct ConvexMpcUpdatableParams {
    q_vec: DVector<f64>,
    a_mat: DMatrix<f64>,
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
        let a_mat = value
            .a_mat
            .ok_or(ModelError::Other("Missing A matrix".into()))?;
        Ok(ConvexMpcUpdatableParams {
            q_vec: DVector::from_vec(q_vec),
            lb_vec,
            ub_vec,
            a_mat,
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

    options: ConvexMpcOptions<S>,
}

impl<S, C> ConvexMpcGeneric<S, C>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
    C: UpdatableController<S> + SteppableController<S>,
    for<'a> C::Params<'a>: From<OSQPBuilder<'a>>,
{
    pub(super) fn new(
        qp_controller: C,
        updatable_params: ConvexMpcUpdatableParams,
        options: ConvexMpcOptions<S>,
    ) -> Result<Self, ModelError> {
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        Ok(ConvexMpcGeneric {
            qp_controller,
            updatable_params,
            options,
            n_steps,
        })
    }

    fn update_mpc(
        &mut self,
        current_state: &ControllerState<S>,
        k: usize,
    ) -> Result<(), ModelError> {
        let ConvexMpcUpdatableParams {
            q_vec,
            lb_vec,
            ub_vec,
            a_mat,
        } = &mut self.updatable_params;

        let state_ref = extend_vector(self.options.general.get_x_ref(), k, k + self.n_steps - 1);
        let x_op: Vec<_> = extend_vector(
            self.options.general.get_x_operating(),
            k,
            k + self.n_steps - 1,
        )
        .iter()
        .map(|x| ControllerState::<S>::from_slice(x.as_slice()))
        .collect();

        let u_op: Vec<_> = extend_vector(
            self.options.general.get_u_operating(),
            k,
            k + self.n_steps - 1,
        )
        .iter()
        .map(|u| ControllerInput::<S>::from_slice(u.as_slice()))
        .collect();

        let general_params = self
            .options
            .get_general()
            .clone()
            .set_u_operating(&u_op)
            .set_x_operating(&x_op);

        self.qp_controller
            .update_bounds(&current_state.to_vector(), lb_vec, ub_vec);
        self.qp_controller.update_q(&state_ref, q_vec);

        self.qp_controller.update_a(a_mat, &general_params)?;

        let builder = OSQPBuilder::new()
            .q_vec(q_vec.clone())
            .bounds_vec(lb_vec.clone(), ub_vec.clone())
            .a_mat(a_mat.clone());
        self.qp_controller.update(builder.into());

        Ok(())
    }
}

impl<S, C> Controller<S> for ConvexMpcGeneric<S, C>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
    C: UpdatableController<S> + SteppableController<S>,
    for<'a> C::Params<'a>: From<OSQPBuilder<'a>>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];
        let dt = self.options.get_general().get_dt();

        let noise_sources =
            NoiseSources::from_stats(self.options.general.get_noise().unwrap_or_default())?;
        let mut current_state = state_from_slice::<S>(
            noise_sources
                .add_noise(x_traj[0].to_vector())?
                .as_mut_slice(),
        );
        x_traj[0] = current_state.clone();

        // results are in r.0 : [u1, x2, u2, ...]
        for k in 0..self.n_steps - 1 {
            // 1- mpc_update
            // update controller
            self.update_mpc(&current_state, k)?;

            let (_, mpc_u_traj) = self.qp_controller.solve(&current_state)?;
            u_traj[k] = mpc_u_traj[0].clone();

            current_state = self
                .qp_controller
                .step(current_state, Some(&u_traj[k]), dt)?;

            x_traj[k + 1] = try_into_noisy_state::<S>(current_state.to_vector(), &noise_sources)?;
            current_state = x_traj[k + 1].clone();
        }
        Ok((x_traj, u_traj))
    }
}
