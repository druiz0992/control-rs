use super::input::DoublePendulumInput;
use super::model::DoublePendulum;
use super::state::DoublePendulumState;
use crate::numeric_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::ModelError;
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::traits::{Dynamics, State};
use crate::physics::{constants as c, energy::Energy};
use crate::utils::Labelizable;
use nalgebra::Vector3;
use std::sync::Arc;

impl Dynamics for DoublePendulum {
    type State = DoublePendulumState;
    type Input = DoublePendulumInput;

    fn dynamics(
        &self,
        s: &DoublePendulumState,
        input: Option<&Self::Input>,
    ) -> DoublePendulumState {
        let [m1, m2, l1, l2, air_resistance_coeff] =
            self.extract(&["m1", "m2", "l1", "l2", "air_resistance_coeff"]);
        let [theta1, omega1, theta2, omega2] = s.extract(&["theta1", "omega1", "theta2", "omega2"]);
        let input = input.unwrap_or(&Self::Input::default()).clone();
        let [u1, u2] = input.extract(&["u1", "u2"]);

        let g = c::GRAVITY;

        let c = (theta1 - theta2).cos();
        let s = (theta1 - theta2).sin();

        let damping1 = -air_resistance_coeff * omega1 * omega1.abs();
        let damping2 = -air_resistance_coeff * omega2 * omega2.abs();

        let dtheta1 = omega1;
        let dω1 = (m2 * g * theta2.sin() * c
            - m2 * s * (l1 * c * omega1.powi(2) + l2 * omega2.powi(2))
            - (m1 + m2) * g * theta1.sin()
            + (u1 + damping1))
            / (l1 * (m1 + m2 * s * s));
        let dtheta2 = omega2;
        let dω2 = ((m1 + m2) * (l1 * omega1.powi(2) * s - g * theta2.sin() + g * theta1.sin() * c)
            + m2 * l2 * omega2.powi(2) * s * c
            + (u2 + damping2))
            / (l2 * (m1 + m2 * s * s));

        DoublePendulumState {
            theta1: dtheta1,
            omega1: dω1,
            theta2: dtheta2,
            omega2: dω2,
        }
    }

    fn energy(&self, s: &DoublePendulumState) -> Option<Energy> {
        let [m1, m2, l1, l2] = self.extract(&["m1", "m2", "l1", "l2"]);
        let [theta1, omega1, theta2, omega2] = s.extract(&["theta1", "omega1", "theta2", "omega2"]);

        let r1 = Vector3::new(l1 * theta1.sin(), 0.0, -l1 * theta1.cos() + 2.0);
        let r2 = Vector3::new(r1.x + l2 * theta2.sin(), 0.0, r1.z - l2 * theta2.cos());
        let v1 = Vector3::new(l1 * omega1 * theta1.cos(), 0.0, l1 * omega1 * theta1.sin());
        let v2 = Vector3::new(
            v1.x + l2 * omega2 * theta2.cos(),
            0.0,
            v1.z + l2 * omega2 * theta2.sin(),
        );

        let kinetic = 0.5 * (m1 * v1.dot(&v1) + m2 * v2.dot(&v2));
        let potential = m1 * c::GRAVITY * r1[2] + m2 * c::GRAVITY * r2[2];

        Some(Energy::new(kinetic, potential))
    }

    fn state_dims(&self) -> (usize, usize) {
        (DoublePendulumState::dim_q(), DoublePendulumState::dim_v())
    }

    fn update(
        &mut self,
        params: &[f64],
        registry: Option<&Arc<ExprRegistry>>,
    ) -> Result<(), ModelError> {
        let [m1, m2, l1, l2, air_resistance_coeff]: [f64; 5] = params
            .try_into()
            .map_err(|_| ModelError::ConfigError("Incorrect number of parameters.".into()))?;
        *self = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, registry);
        Ok(())
    }
}

impl SymbolicDynamics for DoublePendulum {
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector {
        // Define symbolic variables
        let theta1 = state.get(DoublePendulumState::index_of("theta1")).unwrap();
        let omega1 = state.get(DoublePendulumState::index_of("omega1")).unwrap();
        let theta2 = state.get(DoublePendulumState::index_of("theta2")).unwrap();
        let omega2 = state.get(DoublePendulumState::index_of("omega2")).unwrap();
        let u = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();

        let m1 = registry.get_scalar("m1").unwrap();
        let m2 = registry.get_scalar("m2").unwrap();
        let l1 = registry.get_scalar("l1").unwrap();
        let l2 = registry.get_scalar("l2").unwrap();
        let air_resistance_coeff = registry
            .get_scalar(c::AIR_RESISTANCE_COEFF_SYMBOLIC)
            .unwrap();
        let g = registry.get_scalar(c::GRAVITY_SYMBOLIC).unwrap();

        // Common terms
        let c = theta1.sub(&theta2).cos();
        let s = theta1.sub(&theta2).sin();

        let damping1 = air_resistance_coeff
            .scalef(-1.0)
            .mul(&omega1)
            .mul(&omega1.abs());
        let damping2 = air_resistance_coeff
            .scalef(-1.0)
            .mul(&omega2)
            .mul(&omega2.abs());

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
            .add(&damping1.add(&u[0]).wrap())
            .wrap();
        domega1 = domega1
            .div(&m1.add(&m2).mul(&s).mul(&s).wrap().mul(&l1).wrap())
            .wrap();

        let denom = l2.mul(&m1.add(&m2).mul(&s).mul(&s).wrap()).wrap();
        let mut domega2 = l1.mul(&omega1.pow(2.0)).mul(&s).wrap();
        domega2 = domega2.sub(&g.mul(&theta2.sin()).wrap());
        domega2 = domega2.add(&g.mul(&theta1.sin().mul(&c)).wrap()).wrap();
        domega2 = m1.add(&m2).wrap().mul(&domega2);
        domega2 = domega2.add(&m2.mul(&l2).mul(&omega2.pow(2.0)).mul(&s).mul(&c).wrap());
        domega2 = domega2.add(&damping2.add(&u[1]).wrap()).wrap();
        domega2 = domega2.div(&denom).wrap();

        let dtheta2 = omega2.clone();

        // Return as a symbolic vector
        ExprVector::from_vec(vec![dtheta1, domega1, dtheta2, domega2])
    }
}

#[cfg(test)]
mod tests {
    use crate::numeric_services::symbolic::{SymbolicExpr, TryIntoEvalResult};
    use crate::utils::helpers::within_tolerance;

    use super::*;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_dynamics() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.0, 1.0, 0.0, None);
        let state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);

        let new_state = pendulum.dynamics(&state, None);

        assert!(new_state.theta1.abs() < 1e-6);
        assert!(new_state.omega1.abs() < 1e-6);
        assert!(new_state.theta2.abs() < 1e-6);
        assert!(new_state.omega2.abs() < 1e-6);
    }

    #[test]
    #[ignore]
    fn test_dynamics_symbolic() {
        let registry = Arc::new(ExprRegistry::new());
        let double_pendulum = DoublePendulum::new(1.0, 2.0, 0.0, 0.0, 1.0, Some(&registry));
        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let dynamics_func = double_pendulum.dynamics_symbolic(&state_symbol, &registry);
        dbg!(&dynamics_func);
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
            l2 in 0.1f64..5.0,
            air_resistance_coeff in 0.0f64..5.0
        ) {
            let registry = Arc::new(ExprRegistry::new());
            registry.insert_var("theta1", theta1);
            registry.insert_var("theta2", theta2);
            registry.insert_var("omega1", omega1);
            registry.insert_var("omega2", omega2);

            let pendulum = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));

            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();

            let new_state = pendulum.dynamics(&state, None);
            let dynamics_func = pendulum
                .dynamics_symbolic(&state_symbol, &registry)
                .to_fn(&registry)
                .unwrap();
            let new_state_symbol: DoublePendulumState = dynamics_func(None).try_into_eval_result().unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            assert!(within_tolerance(new_state.theta1, new_state_symbol.theta1, tol), "theta1 mismatch");
            assert!(within_tolerance(new_state.omega1, new_state_symbol.omega1, tol), "omega1 mismatch");
            assert!(within_tolerance(new_state.theta2, new_state_symbol.theta2, tol), "theta2 mismatch");
            assert!(within_tolerance(new_state.omega2, new_state_symbol.omega2, tol), "omega2 mismatch");
        }
    }
}
