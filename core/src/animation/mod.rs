use crate::physics::simulator::PhysicsSim;
use async_trait::async_trait;

pub mod macroquad;

#[async_trait]
pub trait Animation {
    type Simulator: PhysicsSim;

    async fn run_animation(mut self, screen_dims: (f32, f32), dt: Option<f64>);
}
