pub mod basic_sim;

use super::{ModelError, traits::Dynamics};
use crate::physics::Energy;

pub use basic_sim::simulator::BasicSim;

pub trait PhysicsSim {
    type Model: Dynamics;

    fn simulate_steps(
        &mut self,
        dt: f64,
        steps: usize,
    ) -> Vec<(f64, <Self::Model as Dynamics>::State, Energy)>;
    fn step(
        &mut self,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<&<Self::Model as Dynamics>::State, ModelError>;
    fn model(&self) -> &Self::Model;
    fn state(&self) -> &<Self::Model as Dynamics>::State;
}
