use std::default;

use crate::{
    controllers::{ControllerInput, ControllerState, options::ControllerOptions},
    physics::traits::PhysicsSim,
};

const DEFAULT_N_STEPS: usize = 20;

pub struct ConvexMpcOptions<S: PhysicsSim> {
    pub n_steps: usize,

    pub general: ControllerOptions<S>,
}

impl<S: PhysicsSim> Default for ConvexMpcOptions<S> {
    fn default() -> Self {
        Self {
            n_steps: DEFAULT_N_STEPS,
            general: ControllerOptions::<S>::default(),
        }
    }
}

impl<S> ConvexMpcOptions<S>
where
    S: PhysicsSim,
{
    pub fn get_general(&self) -> &ControllerOptions<S> {
        &self.general
    }
    pub fn get_n_steps(&self) -> usize {
        self.n_steps
    }

    pub fn set_n_steps(self, n_steps: usize) -> Self {
        let mut new = self;
        new.n_steps = n_steps;
        new
    }

    pub fn set_general(self, general: ControllerOptions<S>) -> Self {
        let mut new = self;
        new.general = general;
        new
    }
}
