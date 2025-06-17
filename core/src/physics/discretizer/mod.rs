pub mod backward_euler;
pub mod forward_euler;
pub mod hermite_simpson;
pub mod implicit_midpoint;
pub mod mid_point;
pub mod rk4;
pub mod utils;
pub mod zero_order_hold;

use std::sync::Arc;

use crate::physics::ModelError;
use crate::physics::traits::Dynamics;
use crate::{numeric_services::symbolic::SymbolicFunction, utils::evaluable::Evaluable};
pub use backward_euler::BackwardEuler;
pub use forward_euler::ForwardEuler;
pub use hermite_simpson::HermiteSimpson;
pub use implicit_midpoint::ImplicitMidpoint;
pub use mid_point::MidPoint;
use nalgebra::DMatrix;
pub use rk4::{RK4, RK4Symbolic};
pub use zero_order_hold::ZOH;

use super::traits::{LinearDynamics, SymbolicDynamics};

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
    fn jacobian_x(&self) -> Result<SymbolicFunction, ModelError>;
    fn jacobian_u(&self) -> Result<SymbolicFunction, ModelError>;
}

pub trait LinearDiscretizer<D: LinearDynamics>: Discretizer<D> {
    fn jacobian_x(&self) -> &DMatrix<f64>;
    fn jacobian_u(&self) -> &DMatrix<f64>;
}

pub type NumericFunction = Box<dyn Fn(&[f64]) -> DMatrix<f64>>;

