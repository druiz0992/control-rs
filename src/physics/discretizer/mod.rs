pub mod backward_euler;
pub mod forward_euler;
pub mod mid_point;
pub mod rk4;
pub mod implicit_midpoint;
pub mod hermite_simpson;

use crate::physics::traits::Dynamics;

pub trait Discretizer<D: Dynamics> {
    fn step(&mut self, dynamics: &D, state: &D::State, dt: f64) -> D::State;
}
