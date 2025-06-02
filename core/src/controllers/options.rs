use crate::{
    controllers::{ControllerInput, ControllerState},
    physics::traits::{PhysicsSim, State},
};

pub struct ControllerOptions<S: PhysicsSim> {
    x_ref: Vec<ControllerState<S>>,
    u_ref: Vec<ControllerInput<S>>,

    /// Operating points => linearization points
    u_op: ControllerInput<S>,
    x_op: ControllerState<S>,

    /// closed loop options
    noise: Option<(f64, f64)>,
    u_limits: Option<(f64, f64)>,
    x_limits: Option<(f64, f64)>,
}

impl<S: PhysicsSim> Clone for ControllerOptions<S> {
    fn clone(&self) -> Self {
        Self {
            x_ref: self.x_ref.clone(),
            u_ref: self.u_ref.clone(),

            u_op: self.u_op.clone(),
            x_op: self.x_op.clone(),

            noise: self.noise,
            u_limits: self.u_limits,
            x_limits: self.x_limits,
        }
    }
}

impl<S: PhysicsSim> Default for ControllerOptions<S> {
    fn default() -> Self {
        Self {
            x_ref: vec![ControllerState::<S>::default(); 1],
            u_ref: vec![ControllerInput::<S>::default(); 1],

            u_op: ControllerInput::<S>::default(),
            x_op: ControllerState::<S>::default(),

            noise: None,
            u_limits: None,
            x_limits: None,
        }
    }
}
impl<S> ControllerOptions<S>
where
    S: PhysicsSim,
{
    pub fn get_u_operating(&self) -> &ControllerInput<S> {
        &self.u_op
    }

    pub fn get_x_operating(&self) -> &ControllerState<S> {
        &self.x_op
    }

    pub fn get_x_ref(&self) -> &[ControllerState<S>] {
        &self.x_ref
    }
    pub fn get_u_ref(&self) -> &[ControllerInput<S>] {
        &self.u_ref
    }

    pub fn get_noise(&self) -> Option<(f64, f64)> {
        self.noise
    }

    pub fn get_u_limits(&self) -> Option<(f64, f64)> {
        self.u_limits
    }
    pub fn get_x_limits(&self) -> Option<(f64, f64)> {
        self.x_limits
    }

    pub fn concatenate_operating_point(&self) -> Vec<f64> {
        let mut vals = self.get_x_operating().to_vec();
        vals.extend(self.get_u_operating().to_vec());

        vals
    }

    pub fn set_u_operating(self, u_op: &ControllerInput<S>) -> Self {
        let mut new = self;
        new.u_op = u_op.clone();
        new
    }

    pub fn set_x_operating(self, x_op: &ControllerState<S>) -> Self {
        let mut new = self;
        new.x_op = x_op.clone();
        new
    }

    pub fn set_x_ref(self, x_ref: &[ControllerState<S>]) -> Self {
        let mut new = self;
        new.x_ref = x_ref.to_owned();
        new
    }

    pub fn set_u_ref(self, u_ref: &[ControllerInput<S>]) -> Self {
        let mut new = self;
        new.u_ref = u_ref.to_owned();
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
