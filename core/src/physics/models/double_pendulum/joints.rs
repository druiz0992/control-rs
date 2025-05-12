use super::model::DoublePendulum;
use super::state::DoublePendulumState;
use crate::{
    common::Labelizable,
    physics::traits::{Renderable, State},
};
use nalgebra::Vector2;

impl Renderable for DoublePendulum {
    type State = DoublePendulumState;

    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>> {
        let (screen_width, screen_height) = screen_dims;
        let origin = Vector2::new(screen_width, screen_height);

        let [l1, l2] = self.extract(&["l1", "l2"]);
        let [theta1, _, theta2, _] = state.as_vec().try_into().unwrap();

        let total_length = (l1 + l2) as f32;

        let scale = 0.5 * screen_height / total_length;

        // Compute the positions in model space (upward is negative y in this system)
        let p1 = origin
            + Vector2::new(
                (l1 * theta1.sin()) as f32 * scale,
                (l1 * theta1.cos()) as f32 * scale,
            );

        let p2 = p1
            + Vector2::new(
                (l2 * theta2.sin()) as f32 * scale,
                (l2 * theta2.cos()) as f32 * scale,
            );

        vec![origin, p1, p2]
    }
}
