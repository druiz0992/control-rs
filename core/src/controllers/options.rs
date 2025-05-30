use crate::{
    controllers::{ControllerInput, ControllerState},
    physics::traits::PhysicsSim,
};

pub struct ControllerOptions<S: PhysicsSim> {
    x_ref: ControllerState<S>,

    /// linearization points
    u_equilibrium: ControllerInput<S>,
    x_equilibrium: ControllerState<S>,

    /// closed loop options
    noise: Option<(f64, f64)>,
    u_limits: Option<(f64, f64)>,
}

impl<S: PhysicsSim> Clone for ControllerOptions<S> {
    fn clone(&self) -> Self {
        Self {
            x_ref: self.x_ref.clone(),

            u_equilibrium: self.u_equilibrium.clone(),
            x_equilibrium: self.x_equilibrium.clone(),

            noise: self.noise.clone(),
            u_limits: self.u_limits.clone(),
        }
    }
}

impl<S: PhysicsSim> Default for ControllerOptions<S> {
    fn default() -> Self {
        Self {
            x_ref: ControllerState::<S>::default(),

            u_equilibrium: ControllerInput::<S>::default(),
            x_equilibrium: ControllerState::<S>::default(),

            noise: None,
            u_limits: None,
        }
    }
}
impl<S> ControllerOptions<S>
where
    S: PhysicsSim,
{
    pub fn get_u_equilibrium(&self) -> &ControllerInput<S> {
        &self.u_equilibrium
    }

    pub fn get_x_equilibrium(&self) -> &ControllerState<S> {
        &self.x_equilibrium
    }

    pub fn get_x_ref(&self) -> &ControllerState<S> {
        &self.x_ref
    }

    pub fn get_noise(&self) -> Option<(f64, f64)> {
        self.noise
    }

    pub fn get_u_limits(&self) -> Option<(f64, f64)> {
        self.u_limits
    }

    pub fn set_u_equilibrium(self, u_equilibrium: &ControllerInput<S>) -> Self {
        let mut new = self;
        new.u_equilibrium = u_equilibrium.clone();
        new
    }

    pub fn set_x_equilibrium(self, x_equilibrium: &ControllerState<S>) -> Self {
        let mut new = self;
        new.x_equilibrium = x_equilibrium.clone();
        new
    }

    pub fn set_x_ref(self, x_ref: &ControllerState<S>) -> Self {
        let mut new = self;
        new.x_ref = x_ref.clone();
        new
    }

    pub fn set_noise(self, noise: (f64, f64)) -> Self {
        let mut new = self;
        new.noise = Some(noise);
        new
    }

    pub fn set_u_limits(self, u_limits: (f64, f64)) -> Self {
        let mut new = self;
        new.u_limits = Some(u_limits);
        new
    }
}
