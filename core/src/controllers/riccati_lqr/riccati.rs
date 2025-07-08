use super::options::RiccatiLQROptions;
use crate::controllers::riccati_lqr::recursion;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, CostFn, JacobianFns, TrajectoryHistory,
    into_clamped_input, try_into_noisy_state,
};
use crate::physics::ModelError;
use crate::physics::discretizer::{LinearDiscretizer, NumericDiscretizer, SymbolicDiscretizer};
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, LinearDynamics, PhysicsSim, State, SymbolicDynamics};
use crate::utils::Labelizable;
use crate::utils::noise::NoiseSources;
use general::helpers::get_or_first;
use nalgebra::{DMatrix, DVector};

pub struct RiccatiRecursion<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,
    k_ss: DMatrix<f64>,
    p_ss: DMatrix<f64>,

    n_steps: usize,

    jacobian_fns: JacobianFns,

    options: RiccatiLQROptions<S>,
}

impl<S> RiccatiRecursion<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics + Labelizable,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new_symbolic(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_u_fn = sim.discretizer().jacobian_u()?;
        let jacobian_x_fn = sim.discretizer().jacobian_x()?;

        let jacobian_fns = JacobianFns::new(jacobian_x_fn, jacobian_u_fn);
        RiccatiRecursion::from_parts(sim, cost_fn, jacobian_fns, options)
    }
}

impl<S> RiccatiRecursion<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: NumericDiscretizer<S::Model>,
{
    pub fn new_numeric(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_u_fn = sim.discretizer().jacobian_u();
        let jacobian_x_fn = sim.discretizer().jacobian_x();

        let jacobian_fns = JacobianFns::new(jacobian_x_fn, jacobian_u_fn);

        RiccatiRecursion::from_parts(sim, cost_fn, jacobian_fns, options)
    }
}
impl<S> RiccatiRecursion<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics + Labelizable,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new_linear(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_u_fn = sim.discretizer().jacobian_u();
        let jacobian_x_fn = sim.discretizer().jacobian_x();

        let jacobian_fns = JacobianFns::new(jacobian_x_fn, jacobian_u_fn);

        RiccatiRecursion::from_parts(sim, cost_fn, jacobian_fns, options)
    }
}

impl<S> RiccatiRecursion<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model>,
{
    fn from_parts(
        sim: S,
        cost_fn: CostFn<S>,
        jacobian_fns: JacobianFns,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        Ok(RiccatiRecursion {
            sim,
            cost_fn,
            options,
            k_ss: DMatrix::default(),
            p_ss: DMatrix::default(),
            n_steps,
            jacobian_fns,
        })
    }

    fn compute_gain(
        &mut self,
        a_mat: &[DMatrix<f64>],
        b_mat: &[DMatrix<f64>],
    ) -> Result<Vec<DMatrix<f64>>, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

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
            let (p_ss, k_ss) = recursion::solve_steady_state_lqr(
                &a_mat[0],
                &b_mat[0],
                &q_mat,
                &r_mat,
                &self.options,
            )?;
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
                let state_mat = get_or_first(a_mat, k);
                let control_mat = get_or_first(b_mat, k);
                let (p, k_gain) =
                    recursion::riccati_recursion(state_mat, control_mat, &q_mat, &r_mat, &p_next)?;
                k_seq[k] = k_gain;
                p_next = p;
            }
            k_seq
        };

        Ok(k_seq)
    }
}

impl<S> Controller<S> for RiccatiRecursion<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: Discretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let n_steps = (self.options.get_general().get_time_horizon()
            / self.options.get_general().get_dt()) as usize
            + 1;
        let (a_mat, b_mat) =
            self.jacobian_fns
                .linearize_full(&self.sim, n_steps, self.options.get_general())?;
        let k_seq = self.compute_gain(&a_mat, &b_mat)?;

        let x_ref = self.options.general.get_x_ref();
        let u_op = self.options.general.get_u_operating();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];
        let dt = self.options.get_general().get_dt();

        // enable noise if configured
        let noise_sources =
            NoiseSources::from_stats(self.options.general.get_noise().unwrap_or_default())
                .map_err(ModelError::Other)?;
        let mut current_state = noise_sources
            .add_noise(x_traj[0].to_vector())
            .map_err(ModelError::Other)?;
        x_traj[0] = ControllerState::<S>::from_slice(current_state.as_slice());

        let u_limits = self.options.general.get_u_limits();

        // Common rollout using `k_seq`
        for k in 0..self.n_steps - 1 {
            let x_reference = get_or_first(x_ref, k);
            let u_operating = get_or_first(u_op, k);
            let current_input = gain_to_input_vector(
                &current_state,
                &k_seq[k],
                &x_reference.to_vector(),
                &u_operating.to_vector(),
            );
            u_traj[k] = into_clamped_input::<S>(current_input, u_limits);

            let x_next = self.sim.step(&x_traj[k], Some(&u_traj[k]), dt)?;
            x_traj[k + 1] = try_into_noisy_state::<S>(x_next.to_vector(), &noise_sources)?;
            current_state = x_traj[k + 1].to_vector();
        }

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
