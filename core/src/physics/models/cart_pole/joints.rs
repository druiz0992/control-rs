use super::model::CartPole;
use super::state::CartPoleState;
use crate::{common::Labelizable, physics::traits::Renderable};
use nalgebra::Vector2;

impl Renderable for CartPole {
    type State = CartPoleState;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>> {
        let (screen_width, screen_height) = screen_dims;
        let origin = Vector2::new(screen_width, screen_height);

        let [l] = self.extract(&["l"]);
        let [pos_x, theta] = state.extract(&["pos_x", "theta"]);

        let total_length = l as f32;

        let scale = 0.5 * screen_height / total_length;

        // Compute the positions in model space (upward is negative y in this system)
        let p1 = origin + Vector2::new(pos_x as f32 * scale, 0.0);

        let p2 = p1
            + Vector2::new(
                (l * theta.sin()) as f32 * scale,
                -(l * theta.cos()) as f32 * scale,
            );

        vec![p1, p2]
    }
}
