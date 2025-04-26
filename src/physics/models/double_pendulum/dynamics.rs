use super::state::DoublePendulumState;
use crate::numeric_services::symbolic::fasteval::{ExprScalar, ExprVector};
use crate::numeric_services::traits::SymbolicExpr;
use crate::physics::traits::{Dynamics, Renderable, State};
use crate::physics::{GRAVITY as G, energy::Energy};
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct DoublePendulum {
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,
}

impl DoublePendulum {
    pub fn new(m1: f64, m2: f64, l1: f64, l2: f64) -> Self {
        DoublePendulum { m1, m2, l1, l2 }
    }

    pub fn parameters(&self) -> (f64, f64, f64, f64) {
        (self.m1, self.m2, self.l1, self.l2)
    }

    pub fn to_symbolic_dynamics(&self) -> Box<dyn SymbolicExpr> {
        // Define symbolic variables
        let theta1 = ExprScalar::new("theta1");
        let omega1 = ExprScalar::new("omega1");
        let theta2 = ExprScalar::new("theta2");
        let omega2 = ExprScalar::new("omega2");

        let m1 = ExprScalar::new("m1");
        let m2 = ExprScalar::new("m2");
        let l1 = ExprScalar::new("l1");
        let l2 = ExprScalar::new("l2");
        let g = ExprScalar::new("g");

        // Common terms
        let c = theta1.sub(&theta2).cos();
        let s = theta1.sub(&theta2).sin();

        // Dynamics equations
        let dtheta1 = omega1.clone();
        let domega1 = m2
            .mul(&g)
            .mul(&theta2.sin())
            .mul(&c)
            .sub(
                &m2.mul(&s).mul(
                    &l1.mul(&c)
                        .mul(&omega1.pow(2.0))
                        .add(&l2.mul(&omega2.pow(2.0))),
                ),
            )
            .sub(&(m1.add(&m2)).mul(&g).mul(&theta1.sin()))
            .div(&l1.mul(&(m1.add(&m2.mul(&s.pow(2.0))))));

        let dtheta2 = omega2.clone();
        let domega2 = (m1.add(&m2))
            .mul(
                &l1.mul(&omega1.pow(2.0))
                    .mul(&s)
                    .sub(&g.mul(&theta2.sin()))
                    .add(&g.mul(&theta1.sin()).mul(&c)),
            )
            .add(&m2.mul(&l2.mul(&omega2.pow(2.0)).mul(&s).mul(&c)))
            .div(&l2.mul(&(m1.add(&m2.mul(&s.pow(2.0))))));

        // Return as a symbolic vector
        Box::new(ExprVector::from_vec(vec![
            dtheta1, domega1, dtheta2, domega2,
        ]))
    }
}

impl Dynamics for DoublePendulum {
    type State = DoublePendulumState;

    fn dynamics(&self, s: &DoublePendulumState) -> DoublePendulumState {
        let (m1, m2, l1, l2) = self.parameters();
        let (θ1, θ̇1, θ2, θ̇2) = s.state();

        let c = (θ1 - θ2).cos();
        let s = (θ1 - θ2).sin();

        let dθ1 = θ̇1;
        let dω1 = (m2 * G * θ2.sin() * c
            - m2 * s * (l1 * c * θ̇1.powi(2) + l2 * θ̇2.powi(2))
            - (m1 + m2) * G * θ1.sin())
            / (l1 * (m1 + m2 * s * s));
        let dθ2 = θ̇2;
        let dω2 = ((m1 + m2) * (l1 * θ̇1.powi(2) * s - G * θ2.sin() + G * θ1.sin() * c)
            + m2 * l2 * θ̇2.powi(2) * s * c)
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
