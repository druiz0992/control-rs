use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::Energy;
use nalgebra::Vector2;
use std::sync::Arc;

pub trait Dynamics: Clone {
    type State: Clone + super::state::State;

    fn dynamics(&self, state: &Self::State, input: Option<&[f64]>) -> Self::State;
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector;
    fn energy(&self, state: &Self::State) -> Energy;
    fn state_dims(&self) -> (usize, usize);

    fn linear_term(&self, _dt: &ExprScalar, _registry: &Arc<ExprRegistry>) -> Option<ExprVector> {
        None
    }
}

pub trait Renderable {
    type State: Clone + super::state::State;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>>;
}
