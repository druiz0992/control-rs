pub mod basic_sim;

use super::{
    ModelError,
    traits::{Discretizer, Dynamics},
};

pub use basic_sim::simulator::BasicSim;

pub trait PhysicsSim {
    type Model: Dynamics;
    type Discretizer: Discretizer<Self::Model>;

    fn rollout(
        &self,
        initial_state: &<Self::Model as Dynamics>::State,
        input: Option<&Vec<<Self::Model as Dynamics>::Input>>,
        dt: f64,
        steps: usize,
    ) -> Result<Vec<<Self::Model as Dynamics>::State>, ModelError>;
    fn step(
        &self,
        state: &<Self::Model as Dynamics>::State,
        input: Option<&<Self::Model as Dynamics>::Input>,
        dt: f64,
    ) -> Result<<Self::Model as Dynamics>::State, ModelError>;
    fn model(&self) -> &Self::Model;
    fn discretizer(&self) -> &Self::Discretizer;
}
