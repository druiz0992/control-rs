use crate::physics::simulator::PhysicsSim;
use async_trait::async_trait;

pub mod macroquad;

#[async_trait]
pub trait Animation {
    type Simulator: PhysicsSim;

    async fn run_animation(self, simulator: &mut Self::Simulator, screen_dims: (f32, f32));
}
