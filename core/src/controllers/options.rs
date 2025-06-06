use super::ConstraintTransform;
use crate::controllers::{ControllerInput, ControllerState};
use crate::physics::ModelError;
use crate::physics::traits::PhysicsSim;
use crate::physics::traits::State;

const DEFAULT_DT: f64 = 0.01;
const DEFAULT_TIME_HORIZON: f64 = 10.0;

/// Represents the configuration options for a controller.
///
/// # Type Parameters
/// - `S`: A type that implements the `PhysicsSim` trait, representing the physics simulation.
///
/// # Fields
/// - `x_ref`: A vector of reference states for the controller.
/// - `u_ref`: A vector of reference inputs for the controller.
/// - `u_op`: The operating point for the controller input, used for linearization.
/// - `x_op`: The operating point for the controller state, used for linearization.
/// - `noise`: Optional noise parameters `(mean, variance)` for the closed-loop system.
/// - `u_limits`: Optional limits `(min, max)` for the controller input.
/// - `x_limits`: Optional limits `(min, max)` for the controller state.
pub struct ControllerOptions<S: PhysicsSim> {
    x_ref: Vec<ControllerState<S>>,
    u_ref: Vec<ControllerInput<S>>,

    time_horizon: f64,
    dt: f64,

    /// Operating points => linearization points
    u_op: ControllerInput<S>,
    x_op: ControllerState<S>,

    /// Estimated params => real model params are those
    /// used to carry out physics. Estimated params are those
    /// used to compute controls. They can differ.
    estimated_params: Option<Vec<f64>>,

    /// closed loop options
    noise: Option<(f64, f64)>,
    u_limits: Option<ConstraintTransform>,
    x_limits: Option<ConstraintTransform>,
}

impl<S: PhysicsSim> Clone for ControllerOptions<S> {
    fn clone(&self) -> Self {
        Self {
            x_ref: self.x_ref.clone(),
            u_ref: self.u_ref.clone(),
            time_horizon: self.time_horizon,
            dt: self.dt,

            u_op: self.u_op.clone(),
            x_op: self.x_op.clone(),

            estimated_params: self.estimated_params.clone(),

            noise: self.noise,
            u_limits: self.u_limits.clone(),
            x_limits: self.x_limits.clone(),
        }
    }
}

impl<S: PhysicsSim> Default for ControllerOptions<S> {
    fn default() -> Self {
        Self {
            x_ref: vec![ControllerState::<S>::default(); 1],
            u_ref: vec![ControllerInput::<S>::default(); 1],
            dt: DEFAULT_DT,
            time_horizon: DEFAULT_TIME_HORIZON,

            u_op: ControllerInput::<S>::default(),
            x_op: ControllerState::<S>::default(),

            estimated_params: None,

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

    pub fn get_u_limits(&self) -> Option<&ConstraintTransform> {
        self.u_limits.as_ref()
    }
    pub fn get_x_limits(&self) -> Option<&ConstraintTransform> {
        self.x_limits.as_ref()
    }
    pub fn get_dt(&self) -> f64 {
        self.dt
    }
    pub fn get_time_horizon(&self) -> f64 {
        self.time_horizon
    }
    pub fn get_estimated_params(&self) -> Option<&Vec<f64>> {
        self.estimated_params.as_ref()
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

    pub fn set_u_limits(self, u_limits: ConstraintTransform) -> Self {
        let mut new = self;
        new.u_limits = Some(u_limits);
        new
    }

    pub fn set_x_limits(self, x_limits: ConstraintTransform) -> Self {
        let mut new = self;
        new.x_limits = Some(x_limits);
        new
    }
    pub fn set_dt(self, dt: f64) -> Result<Self, ModelError> {
        if dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Time configuration needs to be greater than 0.0.".into(),
            ));
        }
        let mut new = self;
        new.dt = dt;
        Ok(new)
    }

    pub fn set_time_horizon(self, time_horizon: f64) -> Result<Self, ModelError> {
        if time_horizon <= 0.0 {
            return Err(ModelError::ConfigError(
                "Time configuration needs to be greater than 0.0.".into(),
            ));
        }
        let mut new = self;
        new.time_horizon = time_horizon;
        Ok(new)
    }

    pub fn set_estimated_params(self, params: Vec<f64>) -> Self {
        let mut new = self;
        new.estimated_params = Some(params);

        new
    }
}
