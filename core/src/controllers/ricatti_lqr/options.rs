const MAX_ITER: usize = 100;
const TOL: f64 = 1e-3;

#[derive(Default)]
pub struct RicattiLQROptions {
    steady_state: bool,
    max_iter: usize,
    tol: f64,
}

impl RicattiLQROptions {
    pub fn enable_inifinte_horizon() -> Self {
        Self {
            steady_state: true,
            max_iter: MAX_ITER,
            tol: TOL,
        }
    }

    pub fn enable_finite_horizon() -> Self {
        Self {
            steady_state: false,
            max_iter: MAX_ITER,
            tol: TOL,
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

    pub fn set_max_iter(&mut self, max_iter: usize) {
        self.max_iter = max_iter;
    }

    pub fn set_tol(&mut self, tol: f64) {
        self.tol = tol;
    }
}
