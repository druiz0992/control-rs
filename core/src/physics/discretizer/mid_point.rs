use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics};

#[derive(Default)]
pub struct MidPoint;

impl MidPoint {
    pub fn new() -> Self {
        Self
    }
}
impl<D> Discretizer<D> for MidPoint
where
    D: Dynamics,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> Result<D::State, ModelError> {
        let x_m = state.clone() + model.dynamics(state) * (dt * 0.5);
        Ok(state.clone() + model.dynamics(&x_m) * dt)
    }
}
impl Describable for MidPoint {
    fn name(&self) -> &'static str {
        "Midpoint"
    }
}
