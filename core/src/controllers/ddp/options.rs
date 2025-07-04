use crate::controllers::ControllerOptions;
use crate::physics::traits::PhysicsSim;

const DEFAULT_MAX_ITERS: usize = 250;
const DEFAULT_MAX_ITERS_LINESEARCH: usize = 20;
const DEFAULT_TOL: f64 = 1e-3;
pub struct DDPOptions<S: PhysicsSim> {
    pub general: ControllerOptions<S>,
    pub ilqr_enable: bool,
    pub max_iters: usize,
    pub max_iters_linesearch: usize,
    pub tol: f64,
    pub verbose: bool,
}

impl<S: PhysicsSim> Default for DDPOptions<S> {
    fn default() -> Self {
        Self {
            general: ControllerOptions::<S>::default(),
            ilqr_enable: false,
            max_iters: DEFAULT_MAX_ITERS,
            max_iters_linesearch: DEFAULT_MAX_ITERS_LINESEARCH,
            tol: DEFAULT_TOL,
            verbose: false,
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

    pub fn get_ilqr_enable(&self) -> bool {
        self.ilqr_enable
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

    pub fn set_general(self, general: ControllerOptions<S>) -> Self {
        let mut new = self;
        new.general = general;
        new
    }

    pub fn set_ilqr_enable(self, enable: bool) -> Self {
        let mut new = self;
        new.ilqr_enable = enable;
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
}
