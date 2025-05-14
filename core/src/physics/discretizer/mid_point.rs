use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics};

#[derive(Default)]
pub struct MidPoint<D: Dynamics> {
    model: D,
}

impl<D: Dynamics> MidPoint<D> {
    pub fn new(model: D) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        Ok(Self { model })
    }
}

impl<D: Dynamics> Discretizer<D> for MidPoint<D> {
    fn step(
        &mut self,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let x_m = state.clone() + self.model.dynamics(state, input) * (dt * 0.5);
        Ok(state.clone() + self.model.dynamics(&x_m, input) * dt)
    }
    fn get_model(&self) -> &D {
        &self.model
    }
}
impl<D: Dynamics> Describable for MidPoint<D> {
    fn name(&self) -> &'static str {
        "Midpoint"
    }
}
