use crate::controllers::ControllerOptions;
use crate::physics::traits::PhysicsSim;
use osqp::Settings;

pub struct QPOptions<S: PhysicsSim> {
    pub general: ControllerOptions<S>,
    pub osqp: Settings,
}

impl<S: PhysicsSim> Default for QPOptions<S> {
    fn default() -> Self {
        Self {
            general: ControllerOptions::<S>::default(),
            osqp: Settings::default(),
        }
    }
}

impl<S> QPOptions<S>
where
    S: PhysicsSim,
{
    pub fn get_general(&self) -> &ControllerOptions<S> {
        &self.general
    }

    pub fn get_osqp_settings(&self) -> &Settings {
        &self.osqp
    }

    pub fn set_general(self, general: ControllerOptions<S>) -> Self {
        let mut new = self;
        new.general = general;
        new
    }

    pub fn set_osqp_settings(self, settings: Settings) -> Self {
        let mut new = self;
        new.osqp = settings;
        new
    }
}

