use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::Energy;
use nalgebra::{DMatrix, Vector2};
use std::sync::Arc;

pub trait Dynamics: Clone {
    type State: Clone + super::state::State;

    fn dynamics(&self, state: &Self::State, input: Option<&[f64]>) -> Self::State;
    fn energy(&self, _state: &Self::State) -> Option<Energy> {
        None
    }
    fn state_dims(&self) -> (usize, usize);
}

pub trait SymbolicDynamics: Dynamics {
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector;
    fn cost_linear_term(
        &self,
        _dt: &ExprScalar,
        _registry: &Arc<ExprRegistry>,
    ) -> Option<ExprVector> {
        None
    }
}

pub trait LinearDynamics: Dynamics {
    fn get_state_slice(&self) -> &DMatrix<f64>;
    fn get_control_slice(&self) -> &DMatrix<f64>;
}

pub trait Renderable: Dynamics {
    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>>;
}
