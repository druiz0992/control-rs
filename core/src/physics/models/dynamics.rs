use crate::numeric_services::symbolic::{ExprMatrix, ExprRegistry, ExprVector};
use crate::physics::Energy;
use nalgebra::Vector2;
use std::sync::Arc;

pub trait Dynamics: Clone {
    type State: Clone + super::state::State;

    fn dynamics(&self, state: &Self::State, input: Option<&[f64]>) -> Self::State;
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector;
    fn energy(&self, state: &Self::State) -> Energy;

    // --- Optional symbolic methods for constrained/contact dynamics ---

    /// Mass matrix M(q)
    fn mass_matrix_symbolic(&self, _registry: &Arc<ExprRegistry>) -> Option<ExprMatrix> {
        None
    }

    /// Constraint Jacobian J(q)
    fn constraint_jacobian_symbolic(&self, _registry: &Arc<ExprRegistry>) -> Option<ExprMatrix> {
        None
    }
}

pub trait Renderable {
    type State: Clone + super::state::State;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>>;
}
