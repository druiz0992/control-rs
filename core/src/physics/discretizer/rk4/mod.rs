pub mod rk4_numeric;
pub mod rk4_symbolic;

use crate::physics::ModelError;
use crate::physics::traits::{Discretizer, Dynamics};
use std::marker::PhantomData;

#[derive(Default, Clone)]
pub struct RK4<D: Dynamics> {
    _phantom_data: PhantomData<D>,
}

impl<D: Dynamics> RK4<D> {
    pub fn new(model: &D) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        Ok(Self {
            _phantom_data: PhantomData,
        })
    }
}

impl<D: Dynamics> Discretizer<D> for RK4<D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let k1 = model.dynamics(state, input);
        let k2 = model.dynamics(&(state.clone() + k1.clone() * (dt * 0.5)), input);
        let k3 = model.dynamics(&(state.clone() + k2.clone() * (dt * 0.5)), input);
        let k4 = model.dynamics(&(state.clone() + k3.clone() * dt), input);

        Ok(state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::discretizer::rk4::rk4_symbolic::RK4Symbolic;
    use crate::physics::models::{DoublePendulum, DoublePendulumState};
    use general::helpers::within_tolerance;
    use proptest::prelude::*;
    use std::f64::consts::PI;
    use std::sync::Arc;
    use symbolic_services::symbolic::ExprRegistry;

    #[test]
    fn test_rk4_step() {
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);
        let rk4 = RK4::new(&dynamics).unwrap();
        let initial_state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);
        let dt = 0.1;

        let next_state = rk4.step(&dynamics, &initial_state, None, dt).unwrap();

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
            l2 in 0.1f64..5.0,
            air_resistance_coeff in 0.0f64..5.0
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let dynamics = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
            let rk4 = RK4::new(&dynamics).unwrap();
            let dt = 0.1;


            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            let next_state_symbolic = rk4_symbolic.step(&dynamics, &state, None, dt).unwrap();
            let next_state = rk4.step(&dynamics, &state, None, dt).unwrap();

            assert!(within_tolerance(next_state.theta1, next_state_symbolic.theta1, tol), "theta1 mismatch");
            assert!(within_tolerance(next_state.omega1, next_state_symbolic.omega1, tol), "omega1 mismatch");
            assert!(within_tolerance(next_state.theta2, next_state_symbolic.theta2, tol), "theta2 mismatch");
            assert!(within_tolerance(next_state.omega2, next_state_symbolic.omega2, tol), "omega2 mismatch");
        }
    }
}
