use super::model::Quadrotor2D;
use crate::{
    physics::traits::{Renderable, State},
    utils::Labelizable,
};
use nalgebra::Vector2;

impl Renderable for Quadrotor2D {
    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>> {
        let (screen_width, screen_height) = screen_dims;
        let origin = Vector2::new(screen_width / 2.0, screen_height / 2.0);

        let [l] = self.extract(&["l"]);
        let [pos_x, pos_y, theta, _, _, _] = state.to_vec().try_into().unwrap();

        let scale = 0.5 * screen_height;

        // === Centering: subtract hover position ===
        // Assume the hover point is at (0, desired_height)
        let hover_y = 1.0; // change as needed
        let centered_x = pos_x;
        let centered_y = pos_y - hover_y;

        // Center the base position
        let base = origin + Vector2::new(centered_x as f32, -centered_y as f32) * scale;

        // Compute rotor positions in body frame and rotate
        let dx = (l / 2.0) * theta.cos();
        let dy = (l / 2.0) * theta.sin();

        let motor1 = base + Vector2::new(-dx as f32, dy as f32) * scale;
        let motor2 = base + Vector2::new(dx as f32, -dy as f32) * scale;

        vec![motor1, base, motor2]
    }
}

