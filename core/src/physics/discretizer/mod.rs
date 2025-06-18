pub mod backward_euler;
pub mod forward_euler;
pub mod hermite_simpson;
pub mod implicit_midpoint;
pub mod mid_point;
pub mod rk4;
pub mod rk4_numeric;
pub mod utils;
pub mod zero_order_hold;

use super::traits::{LinearDynamics, SymbolicDynamics};
use crate::physics::ModelError;
use crate::physics::traits::Dynamics;
use crate::utils::evaluable::EvaluableDMatrix;
use crate::{numeric_services::symbolic::SymbolicFunction, utils::evaluable::Evaluable};
pub use backward_euler::BackwardEuler;
pub use forward_euler::ForwardEuler;
pub use hermite_simpson::HermiteSimpson;
pub use implicit_midpoint::ImplicitMidpoint;
pub use mid_point::MidPoint;
use nalgebra::DMatrix;
pub use rk4::{RK4, RK4Symbolic};
use std::sync::Arc;
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
    fn jacobian_x(&self) -> Result<SymbolicFunction, ModelError>;
    fn jacobian_u(&self) -> Result<SymbolicFunction, ModelError>;
}

pub trait LinearDiscretizer<D: LinearDynamics>: Discretizer<D> {
    fn jacobian_x(&self) -> &DMatrix<f64>;
    fn jacobian_u(&self) -> &DMatrix<f64>;
}

pub trait NumericDiscretizer<D: Dynamics>: Discretizer<D> {
    fn to_numeric_df_dx(&self) -> EvaluableDMatrix;
    fn to_numeric_df_du(&self) -> EvaluableDMatrix;
}

pub struct NumericFunction(pub Arc<dyn for<'a> Fn(&'a [f64]) -> DMatrix<f64> + Send + Sync>);

impl Clone for NumericFunction {
    fn clone(&self) -> Self {
        NumericFunction(Arc::clone(&self.0))
    }
}
impl Evaluable for NumericFunction {
    type Output = DMatrix<f64>;

    fn evaluate(&self, vals: &[f64]) -> Result<Self::Output, ModelError> {
        Ok((self.0)(vals))
    }
}
