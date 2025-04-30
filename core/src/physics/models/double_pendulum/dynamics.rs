use super::model::DoublePendulum;
use super::state::DoublePendulumState;
use crate::numeric_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::traits::Dynamics;
use crate::physics::{GRAVITY as G, energy::Energy};
use nalgebra::Vector3;
use std::sync::Arc;

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

    fn dynamics_symbolic(&self, state: ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector {
        // Define symbolic variables
        let theta1 = state.get(0).unwrap();
        let omega1 = state.get(1).unwrap();
        let theta2 = state.get(2).unwrap();
        let omega2 = state.get(3).unwrap();

        let m1 = registry.get_scalar("m1").unwrap();
        let m2 = registry.get_scalar("m2").unwrap();
        let l1 = registry.get_scalar("l1").unwrap();
        let l2 = registry.get_scalar("l2").unwrap();
        let g = registry.get_scalar("g").unwrap();

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
