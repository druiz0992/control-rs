use crate::{controllers::options::ControllerOptions, physics::traits::PhysicsSim};

const MAX_ITER: usize = 100;
const TOL: f64 = 1e-3;

pub struct RiccatiLQROptions<S: PhysicsSim> {
    /// enable steady state mode
    steady_state: bool,
    max_iter: usize,
    tol: f64,

    pub general: ControllerOptions<S>,
}

impl<S: PhysicsSim> Default for RiccatiLQROptions<S> {
    fn default() -> Self {
        Self {
            steady_state: false,
            max_iter: MAX_ITER,
            tol: TOL,
            general: ControllerOptions::<S>::default(),
        }
    }
}

impl<S> RiccatiLQROptions<S>
where
    S: PhysicsSim,
{
    pub fn enable_infinite_horizon() -> Self {
        let mut options = RiccatiLQROptions::<S>::enable_finite_horizon();
        options.steady_state = true;

        options
    }

    pub fn enable_finite_horizon() -> Self {
        Self {
            steady_state: false,
            max_iter: MAX_ITER,
            tol: TOL,
            general: ControllerOptions::<S>::default(),
        }
    }

    pub fn get_steady_state(&self) -> bool {
        self.steady_state
    }

    pub fn get_max_iter(&self) -> usize {
        self.max_iter
    }

    pub fn get_tol(&self) -> f64 {
        self.tol
    }

    pub fn set_max_iter(self, max_iter: usize) -> Self {
        let mut new = self;
        new.max_iter = max_iter;
        new
    }

    pub fn set_tol(self, tol: f64) -> Self {
        let mut new = self;
        new.tol = tol;
        new
    }

    pub fn set_general(self, general: ControllerOptions<S>) -> Self {
        let mut new = self;
        new.general = general;
        new
    }
}
