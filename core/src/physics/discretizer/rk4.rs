use crate::numeric_services::symbolic::fasteval::{ExprRegistry, ExprScalar};
use crate::numeric_services::symbolic::SymbolicFn;
use crate::numeric_services::traits::SymbolicExpr;
use crate::physics::traits::{Discretizer, Dynamics, FromSymbolicEvalResult, State};
use std::sync::Arc;

pub struct RK4;

impl RK4 {
    pub fn new() -> Self {
        Self
    }
}

impl<D> Discretizer<D> for RK4
where
    D: Dynamics,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let k1 = model.dynamics(state);
        let k2 = model.dynamics(&(state.clone() + k1.clone() * (dt * 0.5)));
        let k3 = model.dynamics(&(state.clone() + k2.clone() * (dt * 0.5)));
        let k4 = model.dynamics(&(state.clone() + k3.clone() * dt));

        let s = state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
        s
    }
}

pub struct RK4Symbolic {
    step_func: SymbolicFn,
    registry: Arc<ExprRegistry>,
}

impl RK4Symbolic {
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Self {
        let state = registry.get_vector("state").unwrap();

        let dt = ExprScalar::new("dt");
        let dt2 = dt.scalef(0.5).wrap();
        let dt6 = dt.scalef(1.0 / 6.0).wrap();

        let k1_s = model.dynamics_symbolic(state.clone()).wrap();
        let k2_s = model
            .dynamics_symbolic(k1_s.scale(&dt2).add(&state).wrap())
            .wrap();
        let k3_s = model
            .dynamics_symbolic(k2_s.scale(&dt2).add(&state).wrap())
            .wrap();
        let k4_s = model
            .dynamics_symbolic(k3_s.scale(&dt).add(&state).wrap())
            .wrap();
        let mut result_s = k1_s
            .add(&k4_s)
            .add(&k2_s.scalef(2.0))
            .add(&k3_s.scalef(2.0))
            .wrap();
        result_s = result_s.scale(&dt6);
        result_s = result_s.add(&state);
        let step_func = result_s.to_fn(registry.as_dyn_registry()).unwrap();
        Self {
            step_func,
            registry,
        }
    }
}

impl<D> Discretizer<D> for RK4Symbolic
where
    D: Dynamics,
{
    fn step(&mut self, _model: &D, state: &D::State, dt: f64) -> D::State {
        self.registry.insert_var("dt".into(), dt);
        let state_vec = state.as_vec();
        let state_components = self.registry.get_vector("state").unwrap().as_vec();
        for (name, value) in state_components.iter().zip(state_vec.iter()) {
            self.registry.insert_var(name.as_str(), *value);
        }
        let result = (self.step_func)(None);
        match result {
            Ok(eval) => FromSymbolicEvalResult::from_symbolic(eval).unwrap(),
            _ => todo!(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::models::{DoublePendulum, DoublePendulumState};
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_rk4_step() {
        let mut rk4 = RK4::new();
        let registry = Arc::new(ExprRegistry::new());
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, registry.clone());
        let initial_state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);
        let dt = 0.1;

        let next_state = rk4.step(&dynamics, &initial_state, dt);

        assert!((next_state.omega1 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_rk4_symbolic_step() {
        let registry = Arc::new(ExprRegistry::new());
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, Arc::clone(&registry));

        let theta1 = 0.0;
        let omega1 = 0.0;
        let theta2 = 0.0;
        let omega2 = 0.0;

        let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
        let mut rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry));
        let dt = 0.1;

        let next_state = rk4_symbolic.step(&dynamics, &state, dt);

        assert!((next_state.omega1 - 0.0).abs() < 1e-6);
    }

    proptest! {
        #[test]
        fn test_symbolic_equivalence(
            theta1 in 0.0..(2.0 * PI),
            theta2 in 0.0..(2.0 * PI),
            omega1 in -5.0..5.0,
            omega2 in -5.0..5.0,
            m1 in 0.1f64..10.0,
            m2 in 0.1f64..10.0,
            l1 in 0.1f64..5.0,
            l2 in 0.1f64..5.0
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let dynamics = DoublePendulum::new(m1, m2, l1, l2, Arc::clone(&registry));
            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let mut rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry));
            let mut rk4 = RK4::new();
            let dt = 0.1;


            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            let next_state_symbolic = rk4_symbolic.step(&dynamics, &state, dt);
            let next_state = rk4.step(&dynamics, &state, dt);

            assert!((next_state.theta1 - next_state_symbolic.theta1).abs() < tol, "theta1 mismatch");
            assert!((next_state.omega1 - next_state_symbolic.omega1).abs() < tol, "omega1 mismatch");
            assert!((next_state.theta2 - next_state_symbolic.theta2).abs() < tol, "theta2 mismatch");
            assert!((next_state.omega2 - next_state_symbolic.omega2).abs() < tol, "omega2 mismatch");
        }
    }
}
