use crate::controllers::{
    Controller, ControllerInput, ControllerOptions, ControllerState, CostFn, InputTrajectory,
    TrajectoryHistory,
};
use crate::physics::ModelError;
use crate::physics::discretizer::{LinearDiscretizer, NumericDiscretizer, SymbolicDiscretizer};
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, LinearDynamics, PhysicsSim, State, SymbolicDynamics};
use crate::utils::evaluable::EvaluableMatrixFn;
use nalgebra::DMatrix;

/// Indirect Shooting controller follows Potryagin's Minimum Principle, which are the first
/// order necessary conditions for a deterministic optimal control problem.
///
/// Given a dynamics function x_n+1 = f(x_n, u_n), and a cost function J, we define the lagrangian L and Hamiltonian H as
///    J = Sum(k:0->N-1) { running_cost(x_k, u_k)} + final_cost(x_N)
///    L = Sum(k:0->N-1) { running_cost(x_k, u_k) + lambda_k+1 * (f(x_k, u_k) - x_k+1)} + final_cost(x_N)
///    H(x,u,lambda) = running_cost(x,u)  + lambda' * f(x,u)
///
///   grad L/lambda_n = grad H/lambda_n - x_n+1 = f(x_n, u_n) - x_n+1 = 0
///   grad L/x_n = grad H/x_n - lambda_n = grad running_cost/x_n + lambda_n+1 * grad f/x_n - lambda_n = 0
///   grad L/x_N = grad final_cost/x_N - lambda_N = 0
///
///   x_n+1 = f(x_n, u_n)
///   lambda_n = grad running_cost/x_n + lambda_n+1 * grad f/x_n
///   lambda_N = grad final_cost/x_N
///   u_n = argmin H(x_n, u_n, lambda_n+1)
///
pub struct IndirectShooting<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,

    u_traj: DMatrix<f64>,

    n_steps: usize,
    dt: f64,

    // df/du, df/dx
    jacobian_u_fn: EvaluableMatrixFn,
    jacobian_x_fn: EvaluableMatrixFn,
}

impl<S> IndirectShooting<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new_linear(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<ControllerOptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = sim.discretizer().jacobian_x();
        let jacobian_u_fn = sim.discretizer().jacobian_u();
        IndirectShooting::from_parts(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)
    }
}

impl<S> IndirectShooting<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new_symbolic(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<ControllerOptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = sim.discretizer().jacobian_x()?;
        let jacobian_u_fn = sim.discretizer().jacobian_u()?;
        IndirectShooting::from_parts(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)
    }
}

impl<S> IndirectShooting<S>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: NumericDiscretizer<S::Model>,
{
    pub fn new_numeric(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<ControllerOptions<S>>,
    ) -> Result<Self, ModelError> {
        let jacobian_x_fn = sim.discretizer().jacobian_x();
        let jacobian_u_fn = sim.discretizer().jacobian_u();
        IndirectShooting::from_parts(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)
    }
}

impl<S> IndirectShooting<S>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
{
    fn from_parts(
        sim: S,
        cost_fn: CostFn<S>,
        jacobian_x_fn: EvaluableMatrixFn,
        jacobian_u_fn: EvaluableMatrixFn,
        options: Option<ControllerOptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();
        let n_steps = (options.get_time_horizon() / options.get_dt()) as usize + 1;

        let u = ControllerInput::<S>::default().to_vector();
        let mut u_traj = DMatrix::zeros(u.len(), n_steps - 1);
        for i in 0..n_steps - 1 {
            u_traj.set_column(i, &u);
        }

        Ok(Self {
            sim,
            cost_fn,
            u_traj,
            n_steps,
            dt: options.get_dt(),
            jacobian_u_fn,
            jacobian_x_fn,
        })
    }

    pub fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        let u_traj = InputTrajectory::<S>::try_from(&self.u_traj)?;
        self.sim
            .rollout(initial_state, Some(u_traj.as_vec()), self.dt, self.n_steps)
    }

    fn backward_pass(&self, states: &[ControllerState<S>]) -> Result<DMatrix<f64>, ModelError> {
        if states.is_empty() {
            return Err(ModelError::ConfigError("Empty states".into()));
        }

        let terminal_state = states.last().unwrap();
        let mut lambda = self.cost_fn.terminal_cost_gradient(terminal_state);

        let input_dims = ControllerInput::<S>::dim_q();
        let r_matrix = self
            .cost_fn
            .get_r()
            .unwrap_or(&DMatrix::zeros(input_dims, input_dims))
            .clone();

        let mut delta_us = DMatrix::zeros(input_dims, self.n_steps - 1);

        for k in (0..self.n_steps - 1).rev() {
            // retrieve state and input values and evaluate jacobians
            let mut vals = states[k].to_vec();
            vals.extend(self.u_traj.column(k).as_slice());
            let jac_dyn_x: DMatrix<f64> = self.jacobian_x_fn.evaluate(&vals)?;
            let jac_dyn_u: DMatrix<f64> = self.jacobian_u_fn.evaluate(&vals)?;

            let rhs = jac_dyn_u.transpose() * &lambda;
            let delta_u = r_matrix
                .clone()
                .lu()
                .solve(&rhs)
                .expect("R matrix must be invertible");

            delta_us.set_column(k, &delta_u);

            let grad_cost_x = self.cost_fn.stage_cost_gradient(&states[k], k)?;
            lambda = &grad_cost_x + jac_dyn_x.transpose() * lambda;
        }

        Ok(-delta_us - &self.u_traj)
    }

    fn step(
        &mut self,
        states: &mut Vec<ControllerState<S>>,
    ) -> Result<(&DMatrix<f64>, f64), ModelError> {
        let u_traj_delta = self.backward_pass(states)?;

        let mut alpha = 1.0;
        let b = 1e-2;

        let old_u_traj = self.u_traj.clone();
        self.u_traj += &u_traj_delta * alpha;
        let mut x_traj = self.rollout(&states[0])?;

        while self.cost_fn.cost(&x_traj, &self.u_traj).unwrap()
            > self.cost_fn.cost(states, &old_u_traj).unwrap()
                - b * alpha * u_traj_delta.iter().map(|x| x * x).sum::<f64>()
        {
            alpha *= 0.5;
            self.u_traj = &old_u_traj + &u_traj_delta * alpha;
            x_traj = self.rollout(&states[0])?;
        }
        *states = x_traj;

        Ok((&self.u_traj, u_traj_delta.abs().max()))
    }
}

impl<S: PhysicsSim> Controller<S> for IndirectShooting<S> {
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let mut states = self.rollout(initial_state)?;
        let mut grad_magnitude = 1.0;

        while grad_magnitude >= 1e-2 {
            let r = self.step(&mut states)?;
            grad_magnitude = r.1;
        }

        let inputs = InputTrajectory::<S>::try_from(&self.u_traj)?;
        let x_traj = self.rollout(initial_state)?;

        Ok((x_traj, inputs.to_vec()))
    }
}
