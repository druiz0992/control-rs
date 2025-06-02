use std::default;

use osqp::Settings;

use crate::{
    controllers::{
        ControllerInput, ControllerState, options::ControllerOptions, qp_lqr::options::QPOptions,
    },
    physics::traits::PhysicsSim,
};

const DEFAULT_N_STEPS: usize = 20;

pub struct ConvexMpcOptions<S: PhysicsSim> {
    pub n_steps: usize,

    pub general: ControllerOptions<S>,
    pub osqp_settings: Settings,
}

impl<S: PhysicsSim> Clone for ConvexMpcOptions<S> {
    fn clone(&self) -> Self {
        Self {
            n_steps: self.n_steps,
            general: self.get_general().clone(),
            osqp_settings: self.get_osqp_settings().clone(),
        }
    }
}

impl<S: PhysicsSim> Default for ConvexMpcOptions<S> {
    fn default() -> Self {
        Self {
            n_steps: DEFAULT_N_STEPS,
            general: ControllerOptions::<S>::default(),
            osqp_settings: Settings::default(),
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
    pub fn get_osqp_settings(&self) -> &Settings {
        &self.osqp_settings
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

    pub fn set_osqp_settings(self, settings: Settings) -> Self {
        let mut new = self;
        new.osqp_settings = settings;
        new
    }
}

impl<S: PhysicsSim> From<ConvexMpcOptions<S>> for QPOptions<S> {
    fn from(value: ConvexMpcOptions<S>) -> Self {
        let general = value.get_general();
        let settings = value.get_osqp_settings();
        QPOptions::<S>::default()
            .set_general(general.clone())
            .set_osqp_settings(settings.clone())
    }
}
