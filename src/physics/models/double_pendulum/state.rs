use crate::physics::traits::State;
use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone)]
pub struct DoublePendulumState {
    pub theta1: f64,
    pub omega1: f64,
    pub theta2: f64,
    pub omega2: f64,
}
impl DoublePendulumState {
    pub fn new(theta1: f64, omega1: f64, theta2: f64, omega2: f64) -> Self {
        DoublePendulumState {
            theta1,
            omega1,
            theta2,
            omega2,
        }
    }

    pub fn state(&self) -> (f64, f64, f64, f64) {
        (self.theta1, self.omega1, self.theta2, self.omega2)
    }
}

impl State for DoublePendulumState {
    fn as_vec(&self) -> Vec<f64> {
        vec![self.theta1, self.omega1, self.theta2, self.omega2]
    }

    fn from_vec(v: Vec<f64>) -> Self {
        Self {
            theta1: v[0],
            omega1: v[1],
            theta2: v[2],
            omega2: v[3],
        }
    }

    fn labels() -> &'static [&'static str] {
        &["theta1", "omega1", "theta2", "omega2"]
    }
}

impl Add for DoublePendulumState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            theta1: self.theta1 + rhs.theta1,
            omega1: self.omega1 + rhs.omega1,
            theta2: self.theta2 + rhs.theta2,
            omega2: self.omega2 + rhs.omega2,
        }
    }
}

impl Sub for DoublePendulumState {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            theta1: self.theta1 - rhs.theta1,
            omega1: self.omega1 - rhs.omega1,
            theta2: self.theta2 - rhs.theta2,
            omega2: self.omega2 - rhs.omega2,
        }
    }
}
impl Mul<f64> for DoublePendulumState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            theta1: self.theta1 * rhs,
            omega1: self.omega1 * rhs,
            theta2: self.theta2 * rhs,
            omega2: self.omega2 * rhs,
        }
    }
}

impl Div<f64> for DoublePendulumState {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            theta1: self.theta1 / rhs,
            omega1: self.omega1 / rhs,
            theta2: self.theta2 / rhs,
            omega2: self.omega2 / rhs,
        }
    }
}
