pub mod backward_euler;
pub mod forward_euler;
pub mod hermite_simpson;
pub mod implicit_midpoint;
pub mod mid_point;
pub mod rk4;
pub mod utils;
pub mod zero_order_hold;

use crate::numeric_services::symbolic::ExprMatrix;
use crate::physics::traits::Dynamics;
use crate::{numeric_services::symbolic::ExprVector, physics::ModelError};
pub use backward_euler::BackwardEuler;
pub use forward_euler::ForwardEuler;
pub use hermite_simpson::HermiteSimpson;
pub use implicit_midpoint::ImplicitMidpoint;
pub use mid_point::MidPoint;
pub use rk4::{RK4, RK4Symbolic};
pub use zero_order_hold::ZOH;

use super::traits::SymbolicDynamics;

pub trait Discretizer<D: Dynamics> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError>;
}

pub trait SymbolicDiscretizer<D: SymbolicDynamics>: Discretizer<D> {
    fn jacobian(&self, unknown: &ExprVector) -> Result<ExprMatrix, ModelError>;
}
