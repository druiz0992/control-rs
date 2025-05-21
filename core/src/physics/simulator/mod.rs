pub mod basic_sim;

use super::{ModelError, traits::Dynamics};
use crate::physics::Energy;

pub use basic_sim::simulator::BasicSim;

pub trait PhysicsSim {
    type Model: Dynamics;

    fn rollout(
        &self,
        initial_state: &<Self::Model as Dynamics>::State,
        dt: f64,
        steps: usize,
    ) -> Vec<(f64, <Self::Model as Dynamics>::State, Energy)>;
    fn step(
        &self,
        state: &<Self::Model as Dynamics>::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<<Self::Model as Dynamics>::State, ModelError>;
    fn model(&self) -> &Self::Model;
}
