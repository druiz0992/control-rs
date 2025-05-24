use crate::controllers::{ControllerInput, ControllerState, CostFn, InputTrajectory};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim, State};
use nalgebra::DMatrix;

pub struct IndirectShootingLQR<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,

    u_traj: DMatrix<f64>,

    n_steps: usize,
    dt: f64,
}

impl<S> IndirectShootingLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(sim: S, cost_fn: CostFn<S>, time_horizon: f64, dt: f64) -> Result<Self, ModelError> {
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

        Ok(Self {
            sim,
            cost_fn,
            u_traj,
            n_steps,
            dt,
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
        let jac_dyn_x = self.sim.discretizer().jacobian_x();
        let jac_dyn_u = self.sim.discretizer().jacobian_u();

        for k in (0..self.n_steps - 1).rev() {
            // retrieve state and input values and evaluate jacobians
            let mut vals = states[k].to_vec();
            vals.extend(self.u_traj.column(k).as_slice());

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
        cost::generic::GenericCost,
        physics::{
            discretizer::ZOH,
            models::{LtiInput, LtiModel, LtiState},
            simulator::BasicSim,
        },
        plotter,
    };
    use nalgebra::dmatrix;

    use super::*;

    #[test]
    fn test_indirect_shooting_lqr() {
        let state_matrix = dmatrix![0.0,1.0; 0.0,0.0];
        let control_matrix = dmatrix![0.0; 1.0];
        let model = LtiModel::<2, 0, 1>::new(state_matrix, control_matrix).unwrap();

        let initial_state = LtiState::<2, 0>::new([1.0, 0.0]);
        let dt = 0.1;
        let sim_time = 10.0;
        let n_steps = (sim_time / dt) as usize + 1;

        let integrator = ZOH::new(&model, dt).unwrap();

        let sim = BasicSim::new(model.clone(), integrator);

        let q_matrix = DMatrix::<f64>::identity(2, 2);
        let qn_matrix = DMatrix::<f64>::identity(2, 2);
        let r_matrix = DMatrix::<f64>::identity(1, 1) * 0.1;
        let zero_x: Vec<_> = (0..n_steps).map(|_| LtiState::default()).collect();
        let cost =
            GenericCost::<_, LtiInput<1, 0>>::new(q_matrix, qn_matrix, r_matrix, zero_x.clone())
                .unwrap();

        let mut controller =
            IndirectShootingLQR::new(sim, Box::new(cost.clone()), sim_time, dt).unwrap();

        controller.solve(&initial_state).unwrap();
        let x_traj = controller.rollout(&initial_state).unwrap();
        let u_traj = controller.get_u_traj();

        let times: Vec<_> = (0..x_traj.len()).map(|i| i as f64 * dt).collect();

        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot1.png").unwrap();

        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();
    }
}
