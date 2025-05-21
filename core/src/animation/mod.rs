use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::simulator::PhysicsSim;
use async_trait::async_trait;

pub mod macroquad;

#[async_trait]
pub trait Animation {
    type Simulator: PhysicsSim;

    async fn run_animation(
        self,
        initial_state: &<<Self::Simulator as PhysicsSim>::Model as Dynamics>::State,
        screen_dims: (f32, f32),
        dt: Option<f64>,
    ) -> Result<(), ModelError>;
}
