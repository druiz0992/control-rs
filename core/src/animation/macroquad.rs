use super::Animation;
use crate::physics::{ModelError, traits::Renderable};
use async_trait::async_trait;
use macroquad::prelude::*;

/// A struct that implements the `Animation` trait using the `macroquad` library for rendering.
///
/// This struct is generic over a type `S` that implements the `PhysicsSim` trait. It uses a
/// `PhantomData` marker to associate itself with the `S` type without actually storing any value of that type.
///
/// # Type Parameters
/// - `S`: A type that implements the `PhysicsSim` trait, representing the physics simulation to be animated.
#[derive(Clone, Debug, Default)]
pub struct Macroquad;

impl Macroquad {
    /// Creates a new instance of the `Macroquad` struct.
    pub fn new() -> Self {
        Self
    }
}

#[async_trait]
impl<M> Animation<M> for Macroquad
where
    M: Renderable + Send + Sync,
    M::State: Clone,
{
    /// Runs the animation loop using the `macroquad` library.
    ///
    /// This method continuously updates the physics simulation, renders the joints of the model,
    /// and draws lines and circles to represent the joints and their connections on the screen.
    ///
    /// # Parameters
    /// - `simulator`: A mutable reference to the physics simulator.
    /// - `screen_dims`: A tuple representing the dimensions of the screen (width, height).
    ///
    /// # Behavior
    /// - Clears the screen with a black background.
    /// - Steps the physics simulation forward by the frame time.
    /// - Renders the joints of the model based on the current state and screen dimensions.
    /// - Draws lines connecting the joints and circles at each joint position.
    /// - Waits for the next frame before repeating the loop.
    async fn run_animation(
        &self,
        model: &M,
        states: &[M::State],
        screen_dims: (f32, f32),
    ) -> Result<(), ModelError> {
        if states.is_empty() {
            return Err(ModelError::ConfigError(
                "State vector cannot be empty".into(),
            ));
        }

        for state in states {
            clear_background(BLACK);
            let joints = model.render_joints(state, screen_dims);

            for win in joints.windows(2) {
                draw_line(win[0].x, win[0].y, win[1].x, win[1].y, 2.0, WHITE);
            }

            for p in &joints[1..] {
                draw_circle(p.x, p.y, 6.0, RED);
            }

            next_frame().await;
        }
        Ok(())
    }
}
