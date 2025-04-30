use crate::physics::traits::{Discretizer, Dynamics};

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
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let x_m = state.clone() + model.dynamics(state) * (dt * 0.5);
        state.clone() + model.dynamics(&x_m) * dt
    }
}
