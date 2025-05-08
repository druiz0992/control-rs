use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics};

#[derive(Default)]
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
    fn step(
        &mut self,
        model: &D,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        Ok(state.clone() + model.dynamics(state, input) * dt)
    }
}

impl Describable for ForwardEuler {
    fn name(&self) -> &'static str {
        "Forward-Euler"
    }
}
