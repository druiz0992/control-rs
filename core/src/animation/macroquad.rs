use super::Animation;
use crate::physics::{
    ModelError,
    traits::{Dynamics, PhysicsSim, Renderable},
};
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
pub struct Macroquad<S: PhysicsSim> {
    sim: S,
}

impl<S> Macroquad<S>
where
    S: PhysicsSim,
{
    /// Creates a new instance of the `Macroquad` struct.
    pub fn new(sim: S) -> Self {
        Self { sim }
    }
}

#[async_trait]
impl<S> Animation for Macroquad<S>
where
    S: PhysicsSim + Send,
    S::Model: Renderable,
    <S::Model as Dynamics>::State: Clone,
{
    /// The associated simulator type, which must implement the `PhysicsSim` trait.
    type Simulator = S;

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
        self,
        initial_state: &<S::Model as Dynamics>::State,
        screen_dims: (f32, f32),
        dt: Option<f64>,
    ) -> Result<(), ModelError> {
        let state = initial_state;
        loop {
            clear_background(BLACK);
            let dt = dt.unwrap_or(get_frame_time() as f64);
            let state = self.sim.step(state, None, dt)?;

            let model = self.sim.model();
            let joints = model.render_joints(&state, screen_dims);

            for win in joints.windows(2) {
                draw_line(win[0].x, win[0].y, win[1].x, win[1].y, 2.0, WHITE);
            }

            for p in &joints[1..] {
                draw_circle(p.x, p.y, 6.0, RED);
            }

            next_frame().await;
        }
    }
}
