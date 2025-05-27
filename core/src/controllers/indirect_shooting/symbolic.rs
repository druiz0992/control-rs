use super::IndirectShootingGeneric;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn};
use crate::physics::ModelError;
use crate::physics::discretizer::SymbolicDiscretizer;
use crate::physics::traits::{PhysicsSim, SymbolicDynamics};

pub struct IndirectShootingSymbolic<S: PhysicsSim>(IndirectShootingGeneric<S>);

impl<S> IndirectShootingSymbolic<S>
where
    S: PhysicsSim,
    S::Model: SymbolicDynamics,
    S::Discretizer: SymbolicDiscretizer<S::Model>,
{
    pub fn new(sim: S, cost_fn: CostFn<S>, time_horizon: f64, dt: f64) -> Result<Self, ModelError> {
        let jacobian_x_fn = Box::new(sim.discretizer().jacobian_x()?);
        let jacobian_u_fn = Box::new(sim.discretizer().jacobian_u()?);

        let controller = IndirectShootingGeneric::<S>::new(
            sim,
            cost_fn,
            jacobian_x_fn,
            jacobian_u_fn,
            time_horizon,
            dt,
        )?;

        Ok(Self(controller))
    }
}

impl<S: PhysicsSim> Controller<S> for IndirectShootingSymbolic<S> {
    fn get_u_traj(&self) -> Vec<ControllerInput<S>> {
        self.0.get_u_traj()
    }

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        self.0.rollout(initial_state)
    }

    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError> {
        self.0.solve(initial_state)
    }
}

/*
#[cfg(test)]
mod tests {
    use crate::{
        animation::{Animation, macroquad::Macroquad},
        cost::terminal_minimum_control::TerminalMinControlCost,
        numeric_services::symbolic::ExprRegistry,
        physics::{
            discretizer::RK4Symbolic,
            models::{DoublePendulum, DoublePendulumInput, DoublePendulumState},
            simulator::BasicSim,
        },
    };
    use nalgebra::DMatrix;
    use std::f64::consts::PI;
    use std::sync::Arc;

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
        let final_state = DoublePendulumState::new(theta1, omega1, theta2, omega2);

        let dt = 0.01;
        let sim_time = 2.0;

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
            IndirectShootingSymbolic::new(sim, Box::new(cost), sim_time, dt).unwrap();

        let mut states = controller.0.rollout(&initial_state).unwrap();
        let mut grad_magnitude = 1.0;
        let mut iter = 0;

        while grad_magnitude >= 1e-2 {
            let r = controller.0.step(&mut states).unwrap();
            grad_magnitude = r.1;

            let x_traj = controller.0.rollout(&initial_state).unwrap();
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
*/
