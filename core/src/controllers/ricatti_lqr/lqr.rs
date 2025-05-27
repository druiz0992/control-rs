use nalgebra::DMatrix;

use super::options::RicattiLQROptions;
use crate::controllers::ricatti_lqr::recursion;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn, InputTrajectory};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim, State};

pub struct RicattiRecursionLQR<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,
    u_traj: InputTrajectory<S>,
    k_seq: Vec<DMatrix<f64>>,

    n_steps: usize,
    dt: f64,

    options: RicattiLQROptions,
}

impl<S> RicattiRecursionLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        time_horizon: f64,
        dt: f64,
        options: RicattiLQROptions,
    ) -> Result<Self, ModelError> {
        if time_horizon <= 0.0 || dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Incorrect time configuration".into(),
            ));
        }
        let n_steps = (time_horizon / dt) as usize + 1;

        let u_traj = InputTrajectory(vec![ControllerInput::<S>::default(); n_steps - 1]);

        Ok(RicattiRecursionLQR {
            sim,
            cost_fn,
            options,
            k_seq: Vec::new(),
            u_traj,
            n_steps,
            dt,
        })
    }

    pub fn rollout_with_noise(
        &self,
        initial_state: &ControllerState<S>,
        noise_std: f64,
    ) -> Vec<ControllerState<S>> {
        use rand_distr::{Distribution, Normal};
        let normal = Normal::new(0.0, noise_std).unwrap();

        let a_mat = self.sim.discretizer().jacobian_x();
        let b_mat = self.sim.discretizer().jacobian_u();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];

        for k in 0..self.n_steps - 1 {
            let x_k_vec = x_traj[k].to_vector();
            let k_gain = &self.k_seq[k];
            let u_k_vec = -k_gain * &x_k_vec;

            // Apply control input
            let noise: DMatrix<f64> = DMatrix::from_fn(x_k_vec.nrows(), 1, |_, _| {
                normal.sample(&mut rand::thread_rng())
            });
            let x_next_vec = a_mat.clone() * x_k_vec + b_mat.clone() * u_k_vec + noise;

            x_traj[k + 1] = ControllerState::<S>::from_vector(x_next_vec);
        }

        x_traj
    }
}

impl<S> Controller<S> for RicattiRecursionLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    fn get_u_traj(&self) -> Vec<ControllerInput<S>> {
        self.u_traj.0.clone()
    }

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        self.sim.rollout(
            initial_state,
            Some(self.u_traj.as_vec()),
            self.dt,
            self.n_steps,
        )
    }

    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];

        let a_mat = self.sim.discretizer().jacobian_x();
        let b_mat = self.sim.discretizer().jacobian_u();

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

        self.k_seq = if self.options.get_steady_state() {
            let k_ss =
                recursion::solve_steady_state_lqr(a_mat, b_mat, &q_mat, &r_mat, &self.options)?;
            vec![k_ss; self.n_steps - 1]
        } else {
            let mut p_mat = self
                .cost_fn
                .get_qn()
                .cloned()
                .unwrap_or(DMatrix::zeros(state_dim, state_dim));
            let mut k_seq = vec![DMatrix::zeros(input_dim, state_dim); self.n_steps - 1];
            for k in (0..self.n_steps - 1).rev() {
                let (p, k_gain) =
                    recursion::ricatti_recursion(a_mat, b_mat, &q_mat, &r_mat, &p_mat)?;
                k_seq[k] = k_gain;
                p_mat = p;
            }
            k_seq
        };

        // Common rollout using `k_seq`
        for k in 0..self.n_steps - 1 {
            let x_k_vec = x_traj[k].to_vector();
            let u_k_vec = -&self.k_seq[k] * &x_k_vec;
            let u_k = ControllerInput::<S>::from_vector(u_k_vec.clone());
            u_traj[k] = u_k;

            let x_next_vec = a_mat * x_k_vec + b_mat * u_k_vec;
            x_traj[k + 1] = ControllerState::<S>::from_vector(x_next_vec);
        }

        self.u_traj = InputTrajectory(u_traj.clone());
        Ok(u_traj)
    }
}
