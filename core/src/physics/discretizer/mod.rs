pub mod backward_euler;
pub mod forward_euler;
pub mod hermite_simpson;
pub mod implicit_midpoint;
pub mod mid_point;
pub mod rk4;
pub mod utils;

use crate::physics::ModelError;
use crate::physics::traits::Dynamics;

pub trait Discretizer<D: Dynamics> {
    fn step(
        &mut self,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError>;
    fn get_model(&self) -> &D;
}

pub use backward_euler::BackwardEuler;
pub use forward_euler::ForwardEuler;
pub use hermite_simpson::HermiteSimpson;
pub use implicit_midpoint::ImplicitMidpoint;
pub use mid_point::MidPoint;
pub use rk4::{RK4, RK4Symbolic};
