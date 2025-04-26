use crate::physics::traits::{Discretizer, Dynamics};

pub struct ForwardEuler;

impl ForwardEuler {
    pub fn new() -> Self {
        Self
    }
}
impl<D> Discretizer<D> for ForwardEuler
where
    D: Dynamics,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        state.clone() + model.dynamics(state) * dt
    }
}
