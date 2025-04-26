use crate::physics::Energy;
use nalgebra::Vector2;

pub trait Dynamics {
    type State: Clone + super::state::State;

    fn dynamics(&self, state: &Self::State) -> Self::State;
    fn energy(&self, state: &Self::State) -> Energy;
}

pub trait Renderable {
    type State: Clone + super::state::State;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>>;
}
