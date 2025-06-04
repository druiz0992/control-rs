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
use nalgebra::DVector;

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

    fn update_mpc(&mut self, current_state: &ControllerState<S>) {
        let ConvexMpcUpdatableParams {
            q_vec,
            lb_vec,
            ub_vec,
        } = &mut self.updatable_params;

        let state_ref = self.options.general.get_x_ref()[0].to_vector();
        self.qp_controller
            .update_bounds(&current_state.to_vector(), lb_vec, ub_vec);
        self.qp_controller.update_q(&state_ref, q_vec);
        let builder = OSQPBuilder::new()
            .q_vec(q_vec.clone())
            .bounds_vec(lb_vec.clone(), ub_vec.clone());
        self.qp_controller.update(builder.into());
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

        let (noise_0_std, noise_std) = self.options.general.get_noise().unwrap_or_default();

        let noise_sources = NoiseSources::from_stats(vec![(0.0, noise_0_std), (0.0, noise_std)])?;
        let mut current_state = state_from_slice::<S>(
            noise_sources
                .add_noise(0, x_traj[0].to_vector())?
                .as_mut_slice(),
        );
        x_traj[0] = current_state.clone();

        // results are in r.0 : [u1, x2, u2, ...]
        for k in 0..self.n_steps - 1 {
            // 1- mpc_update
            // update controller
            self.update_mpc(&current_state);

            let (_, mpc_u_traj) = self.qp_controller.solve(&current_state)?;
            u_traj[k] = mpc_u_traj[0].clone();

            current_state = self
                .qp_controller
                .step(current_state, Some(&u_traj[k]), dt)?;

            x_traj[k + 1] =
                try_into_noisy_state::<S>(current_state.to_vector(), &noise_sources, 1)?;
            current_state = x_traj[k + 1].clone();
        }
        Ok((x_traj, u_traj))
    }
}
