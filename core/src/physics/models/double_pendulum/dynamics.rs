use super::state::DoublePendulumState;
use crate::numeric_services::symbolic::fasteval::{ExprScalar, ExprVector, ExprRegistry};
use crate::numeric_services::traits::{SymbolicExpr, SymbolicRegistry};
use crate::physics::traits::{Dynamics, Renderable, State};
use crate::physics::{GRAVITY as G, energy::Energy};
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize)]
pub struct DoublePendulum {
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,

    #[serde(skip)]
    registry: Arc<ExprRegistry>,
}

impl DoublePendulum {
    pub fn new(m1: f64, m2: f64, l1: f64, l2: f64, registry: Arc<ExprRegistry>) -> Self {
        registry.insert_scalar("m1", ExprScalar::new(m1.to_string()));
        registry.insert_scalar("m2", ExprScalar::new(m2.to_string()));
        registry.insert_scalar("l1", ExprScalar::new(l1.to_string()));
        registry.insert_scalar("l2", ExprScalar::new(l2.to_string()));
        registry.insert_scalar("g", ExprScalar::new(G.to_string()));

        registry.insert_vector(
            "state",
            ExprVector::new(&vec!["theta1", "omega1", "theta2", "omega2"]),
        );

        DoublePendulum {
            m1,
            m2,
            l1,
            l2,
            registry,
        }
    }

    pub fn parameters(&self) -> (f64, f64, f64, f64) {
        (self.m1, self.m2, self.l1, self.l2)
    }
}

impl Dynamics for DoublePendulum {
    type State = DoublePendulumState;

    fn dynamics(&self, s: &DoublePendulumState) -> DoublePendulumState {
        let (m1, m2, l1, l2) = self.parameters();
        let (theta1, omega1, theta2, omega2) = s.state();

        let c = (theta1 - theta2).cos();
        let s = (theta1 - theta2).sin();

        let dθ1 = omega1;
        let dω1 = (m2 * G * theta2.sin() * c
            - m2 * s * (l1 * c * omega1.powi(2) + l2 * omega2.powi(2))
            - (m1 + m2) * G * theta1.sin())
            / (l1 * (m1 + m2 * s * s));
        let dθ2 = omega2;
        let dω2 = ((m1 + m2) * (l1 * omega1.powi(2) * s - G * theta2.sin() + G * theta1.sin() * c)
            + m2 * l2 * omega2.powi(2) * s * c)
            / (l2 * (m1 + m2 * s * s));

        DoublePendulumState {
            theta1: dθ1,
            omega1: dω1,
            theta2: dθ2,
            omega2: dω2,
        }
    }

    fn energy(&self, s: &DoublePendulumState) -> Energy {
        let (m1, m2, l1, l2) = self.parameters();
        let (θ1, θ̇1, θ2, θ̇2) = s.state();

        let r1 = Vector3::new(l1 * θ1.sin(), 0.0, -l1 * θ1.cos() + 2.0);
        let r2 = Vector3::new(r1.x + l2 * θ2.sin(), 0.0, r1.z - l2 * θ2.cos());
        let v1 = Vector3::new(l1 * θ̇1 * θ1.cos(), 0.0, l1 * θ̇1 * θ1.sin());
        let v2 = Vector3::new(v1.x + l2 * θ̇2 * θ2.cos(), 0.0, v1.z + l2 * θ̇2 * θ2.sin());

        let kinetic = 0.5 * (m1 * v1.dot(&v1) + m2 * v2.dot(&v2));
        let potential = m1 * G * r1[2] + m2 * G * r2[2];

        Energy::new(kinetic, potential)
    }

    fn dynamics_symbolic(&self, state: ExprVector) -> ExprVector {
        // Define symbolic variables
        let theta1 = state.get(0).unwrap();
        let omega1 = state.get(1).unwrap();
        let theta2 = state.get(2).unwrap();
        let omega2 = state.get(3).unwrap();

        let m1 = self.registry.get_scalar("m1").unwrap();
        let m2 = self.registry.get_scalar("m2").unwrap();
        let l1 = self.registry.get_scalar("l1").unwrap();
        let l2 = self.registry.get_scalar("l2").unwrap();
        let g = self.registry.get_scalar("g").unwrap();

        // Common terms
        let c = theta1.sub(&theta2).cos();
        let s = theta1.sub(&theta2).sin();

        // Dynamics equations
        let dtheta1 = omega1.clone();
        let mut domega1 = m2.mul(&g).mul(&theta2.sin()).mul(&c);
        domega1 = domega1.sub(
            &m2.mul(&s).mul(
                &l1.mul(&c)
                    .mul(&omega1.pow(2.0).wrap())
                    .add(&l2)
                    .mul(&omega2.pow(2.0).wrap())
                    .wrap(),
            ),
        );
        domega1 = domega1
            .sub(&m1.add(&m2).wrap())
            .mul(&g)
            .mul(&theta1.sin())
            .wrap();
        domega1 = domega1.div(&m1.add(&m2).mul(&s).mul(&s).wrap().mul(&l1).wrap());

        let dtheta2 = omega2.clone();
        let mut domega2 = m1.add(&m2).wrap();
        domega2 = l1
            .mul(&omega1.pow(2.0).wrap())
            .mul(&s)
            .sub(&g)
            .mul(&theta2.sin())
            .add(&g)
            .mul(&theta1.sin())
            .mul(&c)
            .wrap()
            .mul(&domega2);
        domega2 = m2
            .mul(&l2)
            .mul(&omega2.pow(2.0).wrap())
            .mul(&s)
            .mul(&c)
            .add(&domega2)
            .wrap();
        domega2 = domega2.div(&m1.add(&m2).mul(&s).mul(&s).wrap().mul(&l2).wrap());

        // Return as a symbolic vector
        ExprVector::from_vec(vec![dtheta1, domega1, dtheta2, domega2])
    }
}

impl Renderable for DoublePendulum {
    type State = DoublePendulumState;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>> {
        let (screen_width, screen_height) = screen_dims;
        let origin = Vector2::new(screen_width, screen_height);

        let (_, _, l1, l2) = self.parameters();
        let [theta1, _, theta2, _] = state.as_vec().try_into().unwrap();

        let total_length = (l1 + l2) as f32;

        let scale = 0.5 * screen_height / total_length;

        // Compute the positions in model space (upward is negative y in this system)
        let p1 = origin
            + Vector2::new(
                (l1 * theta1.sin()) as f32 * scale,
                -(l1 * theta1.cos()) as f32 * scale,
            );

        let p2 = p1
            + Vector2::new(
                (l2 * theta2.sin()) as f32 * scale,
                -(l2 * theta2.cos()) as f32 * scale,
            );

        vec![origin, p1, p2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{constants, models::state::FromSymbolicEvalResult};
    use approx::assert_abs_diff_eq;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_double_pendulum_new() {
        let registry = Arc::new(ExprRegistry::new());
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, registry.clone());

        assert_eq!(pendulum.m1, 1.0);
        assert_eq!(pendulum.m2, 2.0);
        assert_eq!(pendulum.l1, 1.5);
        assert_eq!(pendulum.l2, 2.5);
    }

    #[test]
    fn test_double_pendulum_parameters() {
        let registry = Arc::new(ExprRegistry::new());
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, registry.clone());

        let params = pendulum.parameters();
        assert_eq!(params, (1.0, 2.0, 1.5, 2.5));
    }

    #[test]
    fn test_dynamics() {
        let registry = Arc::new(ExprRegistry::new());
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.0, 1.0, registry.clone());
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

            let pendulum = DoublePendulum::new(m1, m2, l1, l2, Arc::clone(&registry));

            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let state_symbol = registry.get_vector("state").unwrap();

            let new_state = pendulum.dynamics(&state);
            let dynamics_func = pendulum
                .dynamics_symbolic(state_symbol)
                .to_fn(registry.clone())
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
