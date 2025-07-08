use osqp::Settings;

use crate::controllers::ControllerOptions;
use crate::physics::traits::PhysicsSim;

const DEFAULT_MAX_ITERS: usize = 450;
const DEFAULT_MAX_ITERS_LINESEARCH: usize = 20;
const DEFAULT_TOL: f64 = 1e-5;
pub struct DDPOptions<S: PhysicsSim> {
    pub general: ControllerOptions<S>,
    pub ddp_enable: bool,
    pub max_iters: usize,
    pub max_iters_linesearch: usize,
    pub tol: f64,
    pub verbose: bool,
    pub osqp_settings: Settings,
}

impl<S: PhysicsSim> Default for DDPOptions<S> {
    fn default() -> Self {
        let osqp_settings = Settings::default().verbose(false);
        Self {
            general: ControllerOptions::<S>::default(),
            ddp_enable: false,
            max_iters: DEFAULT_MAX_ITERS,
            max_iters_linesearch: DEFAULT_MAX_ITERS_LINESEARCH,
            tol: DEFAULT_TOL,
            verbose: false,
            osqp_settings,
        }
    }
}

impl<S> DDPOptions<S>
where
    S: PhysicsSim,
{
    pub fn get_general(&self) -> &ControllerOptions<S> {
        &self.general
    }

    pub fn get_ddp_enable(&self) -> bool {
        self.ddp_enable
    }

    pub fn get_max_iters(&self) -> usize {
        self.max_iters
    }

    pub fn get_max_iters_linesearch(&self) -> usize {
        self.max_iters_linesearch
    }

    pub fn get_tol(&self) -> f64 {
        self.tol
    }

    pub fn get_verbose(&self) -> bool {
        self.verbose
    }

    pub fn get_osqp_settings(&self) -> Settings {
        self.osqp_settings.clone()
    }

    pub fn set_general(self, general: ControllerOptions<S>) -> Self {
        let mut new = self;
        new.general = general;
        new
    }

    pub fn set_ddp_enable(self, enable: bool) -> Self {
        let mut new = self;
        new.ddp_enable = enable;
        new
    }

    pub fn set_max_iters(self, max_iters: usize) -> Self {
        let mut new = self;
        new.max_iters = max_iters;
        new
    }

    pub fn set_max_iters_linesearch(self, max_iters: usize) -> Self {
        let mut new = self;
        new.max_iters_linesearch = max_iters;
        new
    }

    pub fn set_tol(self, tol: f64) -> Self {
        let mut new = self;
        new.tol = tol;
        new
    }

    pub fn set_verbose(self, flag: bool) -> Self {
        let mut new = self;
        new.verbose = flag;
        new
    }

    pub fn set_osqp_settings(self, settings: Settings) -> Self {
        let mut new = self;
        new.osqp_settings = settings;
        new
    }
}
