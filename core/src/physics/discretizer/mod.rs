pub mod backward_euler;
pub mod forward_euler;
pub mod hermite_simpson;
pub mod implicit_midpoint;
pub mod mid_point;
pub mod rk4;
pub mod utils;
pub mod zero_order_hold;

use super::traits::{LinearDynamics, SymbolicDynamics};
use crate::physics::ModelError;
use crate::physics::traits::Dynamics;
use crate::utils::evaluable::EvaluableMatrixFn;
pub use backward_euler::BackwardEuler;
pub use forward_euler::ForwardEuler;
pub use hermite_simpson::HermiteSimpson;
pub use implicit_midpoint::ImplicitMidpoint;
pub use mid_point::MidPoint;
pub use rk4::{RK4, rk4_numeric::RK4Numeric, rk4_symbolic::RK4Symbolic};
pub use zero_order_hold::ZOH;

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
    fn jacobian_x(&self) -> Result<EvaluableMatrixFn, ModelError>;
    fn jacobian_u(&self) -> Result<EvaluableMatrixFn, ModelError>;
}

pub trait LinearDiscretizer<D: LinearDynamics>: Discretizer<D> {
    fn jacobian_x(&self) -> EvaluableMatrixFn;
    fn jacobian_u(&self) -> EvaluableMatrixFn;
}

pub trait NumericDiscretizer<D: Dynamics>: Discretizer<D> {
    fn jacobian_x(&self) -> EvaluableMatrixFn;
    fn jacobian_u(&self) -> EvaluableMatrixFn;
}

/// Generation of code that converts Discretizer symbolic expression into numeric function
pub trait CodeGenerator<D: SymbolicDynamics>: SymbolicDiscretizer<D> {
    fn to_numeric_jacobian_x(&self) -> Result<(), ModelError>;
    fn to_numeric_jacobian_u(&self) -> Result<(), ModelError>;
}
