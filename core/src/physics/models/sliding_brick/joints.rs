use super::model::SlidingBrick;
use super::state::SlidingBrickState;
use crate::physics::traits::{Renderable, State};
use nalgebra::Vector2;

impl Renderable for SlidingBrick {
    type State = SlidingBrickState;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>> {
        let (screen_width, screen_height) = screen_dims;
        let origin = Vector2::new(screen_width, screen_height);

        let [pos_x, pos_y, _, _] = state.as_vec().try_into().unwrap();

        let scale = 0.5 * screen_height;

        // Compute the positions in model space (upward is negative y in this system)
        let p1 = origin + Vector2::new(pos_x as f32 * scale, pos_y as f32);

        vec![p1, p1]
    }
}
