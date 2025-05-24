use crate::controllers::{ControllerInput, ControllerState, CostFn, InputTrajectory};
use crate::numeric_services::symbolic::{
    ExprRegistry, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use crate::physics::ModelError;
use crate::physics::constants as c;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, State, SymbolicDynamics};
use nalgebra::DMatrix;
use std::sync::Arc;

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

pub struct IndirectShooting<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,

    u_traj: DMatrix<f64>,

    n_steps: usize,
    dt: f64,

    // df/du, df/dx
    jacobian_u_fn: SymbolicFunction,
    jacobian_x_fn: SymbolicFunction,
}

impl<S> IndirectShooting<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        time_horizon: f64,
        dt: f64,
        registry: &Arc<ExprRegistry>,
    ) -> Result<Self, ModelError> {
        if time_horizon <= 0.0 || dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Incorrect time configuration".into(),
            ));
        }
        let n_steps = (time_horizon / dt) as usize + 1;

        let u = ControllerInput::<S>::default().to_vector();
        let mut u_traj = DMatrix::zeros(u.len(), n_steps - 1);
        for i in 0..n_steps - 1 {
            u_traj.set_column(i, &u);
        }

        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol);

        let jacobian_x = sim.discretizer().jacobian(&state_symbol)?.to_fn(registry)?;
        let jacobian_x_fn = SymbolicFunction::new(jacobian_x, &jacobian_symbols);
        let jacobian_u = sim.discretizer().jacobian(&input_symbol)?.to_fn(registry)?;
        let jacobian_u_fn = SymbolicFunction::new(jacobian_u, &jacobian_symbols);

        Ok(Self {
            sim,
            cost_fn,
            u_traj,
            n_steps,
            dt,
            jacobian_u_fn,
            jacobian_x_fn,
        })
    }

    pub fn get_u_traj(&self) -> Vec<ControllerInput<S>> {
        let traj = InputTrajectory::<S>::try_from(&self.u_traj).unwrap();
        traj.0.clone()
    }

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        let u_traj = InputTrajectory::<S>::try_from(&self.u_traj)?;
        self.sim
            .rollout(initial_state, Some(u_traj.as_vec()), self.dt, self.n_steps)
    }

    fn backward_pass(
        &self,
        states: &[ControllerState<S>],
    ) -> Result<DMatrix<f64>, ModelError> {
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
            let jac_dyn_x: DMatrix<f64> = self.jacobian_x_fn.eval(&vals).try_into_eval_result()?;
            let jac_dyn_u: DMatrix<f64> = self.jacobian_u_fn.eval(&vals).try_into_eval_result()?;

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

    pub fn step(
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
                - b * alpha * &u_traj_delta.iter().map(|x| x * x).sum::<f64>()
        {
            alpha *= 0.5;
            self.u_traj = &old_u_traj + &u_traj_delta * alpha;
            x_traj = self.rollout(&states[0])?;
        }
        *states = x_traj;

        Ok((&self.u_traj, u_traj_delta.abs().max()))
    }

    pub fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError> {
        let mut states = self.rollout(initial_state)?;
        let mut grad_magnitude = 1.0;

        while grad_magnitude >= 1e-2 {
            let r = self.step(&mut states)?;
            grad_magnitude = r.1;
        }

        let inputs = InputTrajectory::<S>::try_from(&self.u_traj)?;

        Ok(inputs.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        animation::{Animation, macroquad::Macroquad},
        cost::terminal_minimum_control::TerminalMinControlCost,
        physics::{
            discretizer::RK4Symbolic,
            models::{DoublePendulum, DoublePendulumInput, DoublePendulumState},
            simulator::BasicSim,
        },
    };
    use std::f64::consts::PI;

    use super::*;

    #[macroquad::test("Double Pendulum Indirect Sh0oting")]
    async fn test_indirect_shooting() {
        let m1 = 1.0;
        let m2 = 1.0;
        let l1 = 1.0;
        let l2 = 1.0;
        let air_resistance_coeff = 0.0;

        let theta1 = PI / 1.6;
        let omega1 = 0.0;
        let theta2 = PI / 1.8;
        let omega2 = 0.0;

        let registry = Arc::new(ExprRegistry::new());
        let initial_state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
        let final_state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);

        let dt = 0.01;

        let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
        let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();

        let sim = BasicSim::new(model.clone(), integrator);

        let qn_matrix = DMatrix::<f64>::identity(4, 4) * 50.0;
        let r_matrix = DMatrix::<f64>::identity(2, 2);

        let cost =
            TerminalMinControlCost::<_, DoublePendulumInput>::new(qn_matrix, r_matrix, final_state)
                .unwrap();

        let animation_sim = Macroquad::new();
        let mut controller =
            IndirectShooting::new(sim, Box::new(cost), 5.0, dt, &registry).unwrap();

        let mut states = controller.rollout(&initial_state).unwrap();
        let mut grad_magnitude = 1.0;
        let mut iter = 0;

        while grad_magnitude >= 1e-2 {
            let r = controller.step(&mut states).unwrap();
            grad_magnitude = r.1;

            let x_traj = controller.rollout(&initial_state).unwrap();
            dbg!(&x_traj[0..10]);

            if iter % 10 == 0 {
                animation_sim
                    .run_animation(&model, &x_traj, (400.0, 300.0))
                    .await
                    .unwrap();
            }
            iter += 1;
        }
    }
}
