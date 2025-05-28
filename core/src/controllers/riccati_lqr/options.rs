use crate::{
    controllers::{ControllerInput, ControllerState},
    physics::traits::PhysicsSim,
};

const MAX_ITER: usize = 100;
const TOL: f64 = 1e-3;

#[derive(Default)]
pub struct RiccatiLQROptions<S: PhysicsSim> {
    steady_state: bool,
    max_iter: usize,
    tol: f64,
    x_ref: ControllerState<S>,
    u_equilibrium: ControllerInput<S>,
}

impl<S> RiccatiLQROptions<S>
where
    S: PhysicsSim,
{
    pub fn enable_inifinte_horizon() -> Self {
        Self {
            steady_state: true,
            max_iter: MAX_ITER,
            tol: TOL,
            x_ref: ControllerState::<S>::default(),
            u_equilibrium: ControllerInput::<S>::default(),
        }
    }

    pub fn enable_finite_horizon() -> Self {
        Self {
            steady_state: false,
            max_iter: MAX_ITER,
            tol: TOL,
            x_ref: ControllerState::<S>::default(),
            u_equilibrium: ControllerInput::<S>::default(),
        }
    }

    pub fn get_u_equilibrium(&self) -> &ControllerInput<S> {
        &self.u_equilibrium
    }

    pub fn get_x_ref(&self) -> &ControllerState<S> {
        &self.x_ref
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

    pub fn set_u_equilibrium(&mut self, u_equilibrium: &ControllerInput<S>) {
        self.u_equilibrium = u_equilibrium.clone();
    }

    pub fn set_x_ref(&mut self, x_ref: &ControllerState<S>) {
        self.x_ref = x_ref.clone();
    }
}
