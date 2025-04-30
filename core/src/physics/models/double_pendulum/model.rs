use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::GRAVITY as G;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DoublePendulum {
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,
}

impl DoublePendulum {
    pub fn new(m1: f64, m2: f64, l1: f64, l2: f64, registry: Option<&Arc<ExprRegistry>>) -> Self {
        if let Some(registry) = registry {
            registry.insert_scalar("m1", ExprScalar::new(m1.to_string()));
            registry.insert_scalar("m2", ExprScalar::new(m2.to_string()));
            registry.insert_scalar("l1", ExprScalar::new(l1.to_string()));
            registry.insert_scalar("l2", ExprScalar::new(l2.to_string()));
            registry.insert_scalar("g", ExprScalar::new(G.to_string()));

            registry.insert_vector(
                "state",
                ExprVector::new(&["theta1", "omega1", "theta2", "omega2"]),
            );
        }

        DoublePendulum { m1, m2, l1, l2 }
    }

    pub fn parameters(&self) -> (f64, f64, f64, f64) {
        (self.m1, self.m2, self.l1, self.l2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::numeric_services::symbolic::{ExprRegistry, SymbolicExpr};
    use crate::physics::models::DoublePendulumState;
    use crate::physics::models::dynamics::Dynamics;
    use crate::physics::models::state::FromSymbolicEvalResult;
    use proptest::prelude::*;
    use std::f64::consts::PI;
    use std::sync::Arc;

    #[test]
    fn test_double_pendulum_new() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, None);
        let (m1, m2, l1, l2) = pendulum.parameters();

        assert_eq!(m1, 1.0);
        assert_eq!(m2, 2.0);
        assert_eq!(l1, 1.5);
        assert_eq!(l2, 2.5);
    }

    #[test]
    fn test_double_pendulum_parameters() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, None);

        let params = pendulum.parameters();
        assert_eq!(params, (1.0, 2.0, 1.5, 2.5));
    }

    #[test]
    fn test_dynamics() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.0, 1.0, None);
        let state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);

        let new_state = pendulum.dynamics(&state);

        assert!(new_state.theta1.abs() < 1e-6);
        assert!(new_state.omega1.abs() < 1e-6);
        assert!(new_state.theta2.abs() < 1e-6);
        assert!(new_state.omega2.abs() < 1e-6);
    }

    proptest! {
        #[test]
        fn test_symbolic_dynamics_randomized(
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
            registry.insert_var("theta1", theta1);
            registry.insert_var("theta2", theta2);
            registry.insert_var("omega1", omega1);
            registry.insert_var("omega2", omega2);

            let pendulum = DoublePendulum::new(m1, m2, l1, l2, Some(&registry));

            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let state_symbol = registry.get_vector("state").unwrap();

            let new_state = pendulum.dynamics(&state);
            let dynamics_func = pendulum
                .dynamics_symbolic(state_symbol, &registry)
                .to_fn(&registry)
                .unwrap();
            let new_state_symbol: DoublePendulumState = FromSymbolicEvalResult::from_symbolic(dynamics_func(None).unwrap()).unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            assert!((new_state.theta1 - new_state_symbol.theta1).abs() < tol, "theta1 mismatch");
            assert!((new_state.omega1 - new_state_symbol.omega1).abs() < tol, "omega1 mismatch");
            assert!((new_state.theta2 - new_state_symbol.theta2).abs() < tol, "theta2 mismatch");
            assert!((new_state.omega2 - new_state_symbol.omega2).abs() < tol, "omega2 mismatch");
        }
    }
}
