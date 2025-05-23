use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::Renderable;
use async_trait::async_trait;

pub mod macroquad;

#[async_trait]
pub trait Animation<M: Renderable + Send + Sync> {
    async fn run_animation(
        &self,
        model: &M,
        states: &[<M as Dynamics>::State],
        screen_dims: (f32, f32),
    ) -> Result<(), ModelError>;
}
