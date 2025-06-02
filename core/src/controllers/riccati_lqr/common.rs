use super::options::RiccatiLQROptions;
use crate::controllers::riccati_lqr::recursion;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, CostFn, InputTrajectory, TrajectoryHistory,
    into_clamped_input, try_into_noisy_state,
};
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim, State};
use crate::utils::evaluable::EvaluableDMatrix;
use crate::utils::noise::NoiseSources;
use nalgebra::{DMatrix, DVector};

const ZERO_MEAN: f64 = 0.0;

pub struct RiccatiRecursionGeneric<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,
    u_traj: InputTrajectory<S>,
    k_ss: DMatrix<f64>,
    p_ss: DMatrix<f64>,

    n_steps: usize,
    dt: f64,

    jacobian_u_fn: EvaluableDMatrix,
    jacobian_x_fn: EvaluableDMatrix,

    options: RiccatiLQROptions<S>,
}

impl<S> RiccatiRecursionGeneric<S>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        jacobian_x_fn: EvaluableDMatrix,
        jacobian_u_fn: EvaluableDMatrix,
        time_horizon: f64,
        dt: f64,
        options: RiccatiLQROptions<S>,
    ) -> Result<Self, ModelError> {
        if time_horizon <= 0.0 || dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Incorrect time configuration".into(),
            ));
        }
        let n_steps = (time_horizon / dt) as usize + 1;

        let u_traj = InputTrajectory::new(vec![ControllerInput::<S>::default(); n_steps - 1]);

        Ok(RiccatiRecursionGeneric {
            sim,
            cost_fn,
            options,
            k_ss: DMatrix::default(),
            p_ss: DMatrix::default(),
            u_traj,
            n_steps,
            jacobian_x_fn,
            jacobian_u_fn,
            dt,
        })
    }

    fn compute_gain(&mut self) -> Result<Vec<DMatrix<f64>>, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let vals = self.options.general.concatenate_operating_point();

        let a_mat = self.jacobian_x_fn.evaluate(&vals)?;
        let b_mat = self.jacobian_u_fn.evaluate(&vals)?;

        let q_mat = self
            .cost_fn
            .get_q()
            .cloned()
            .unwrap_or(DMatrix::zeros(state_dim, state_dim));

        let r_mat = self
            .cost_fn
            .get_r()
            .cloned()
            .unwrap_or(DMatrix::zeros(input_dim, input_dim));

        let k_seq = if self.options.get_steady_state() {
            let (p_ss, k_ss) =
                recursion::solve_steady_state_lqr(&a_mat, &b_mat, &q_mat, &r_mat, &self.options)?;
            self.k_ss = k_ss.clone();
            self.p_ss = p_ss.clone();
            vec![k_ss; self.n_steps - 1]
        } else {
            let mut p_next = self
                .cost_fn
                .get_qn()
                .cloned()
                .unwrap_or(DMatrix::zeros(state_dim, state_dim));
            let mut k_seq = vec![DMatrix::zeros(input_dim, state_dim); self.n_steps - 1];
            for k in (0..self.n_steps - 1).rev() {
                let (p, k_gain) =
                    recursion::riccati_recursion(&a_mat, &b_mat, &q_mat, &r_mat, &p_next)?;
                k_seq[k] = k_gain;
                p_next = p;
            }
            k_seq
        };

        Ok(k_seq)
    }
}

impl<S> Controller<S> for RiccatiRecursionGeneric<S>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let k_seq = self.compute_gain()?;

        // TODO >>> review
        let x_ref = self.options.general.get_x_ref()[0].to_vector();
        let u_op = self.options.general.get_u_operating().to_vector();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];

        // enable noise if configured
        let (noise_0_std, noise_std) = self.options.general.get_noise().unwrap_or_default();

        let noise_sources =
            NoiseSources::from_stats(vec![(ZERO_MEAN, noise_0_std), (ZERO_MEAN, noise_std)])?;
        let mut current_state = noise_sources.add_noise(0, x_traj[0].to_vector())?;

        let u_limits = self.options.general.get_u_limits();

        // Common rollout using `k_seq`
        for k in 0..self.n_steps - 1 {
            let current_input = gain_to_input_vector(&current_state, &k_seq[k], &x_ref, &u_op);
            u_traj[k] = into_clamped_input::<S>(current_input, u_limits);

            let x_next = self.sim.step(&x_traj[k], Some(&u_traj[k]), self.dt)?;
            x_traj[k + 1] = try_into_noisy_state::<S>(x_next.to_vector(), &noise_sources, 1)?;
            current_state = x_traj[k + 1].to_vector();
        }

        self.u_traj = InputTrajectory::new(u_traj.clone());

        Ok((x_traj, u_traj))
    }
}

#[inline]
fn gain_to_input_vector(
    x_current: &DVector<f64>,
    k_gain: &DMatrix<f64>,
    x_ref: &DVector<f64>,
    u_op: &DVector<f64>,
) -> DVector<f64> {
    u_op - k_gain * (x_current - x_ref)
}
