use crate::{
    controllers::{options::ControllerOptions, qp_lqr::options::QPOptions},
    physics::traits::PhysicsSim,
};
use osqp::Settings;

const DEFAULT_FINITIE_HORIZON: f64 = 1.0;

pub struct ConvexMpcOptions<S: PhysicsSim> {
    pub finite_horizon: f64,

    pub general: ControllerOptions<S>,
    pub osqp_settings: Settings,
}

impl<S: PhysicsSim> Clone for ConvexMpcOptions<S> {
    fn clone(&self) -> Self {
        Self {
            finite_horizon: self.finite_horizon,
            general: self.get_general().clone(),
            osqp_settings: self.get_osqp_settings().clone(),
        }
    }
}

impl<S: PhysicsSim> Default for ConvexMpcOptions<S> {
    fn default() -> Self {
        Self {
            finite_horizon: DEFAULT_FINITIE_HORIZON,
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
    pub fn get_horizon(&self) -> f64 {
        self.finite_horizon
    }
    pub fn get_osqp_settings(&self) -> &Settings {
        &self.osqp_settings
    }

    pub fn set_horizon(self, finite_horizon: f64) -> Self {
        let mut new = self;
        new.finite_horizon = finite_horizon;
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
