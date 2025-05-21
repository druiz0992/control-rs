use std::marker::PhantomData;

use crate::physics::ModelError;
use crate::physics::traits::{Discretizer, Dynamics};

#[derive(Default)]
pub struct MidPoint<D: Dynamics> {
    _phantom_data: PhantomData<D>,
}

impl<D: Dynamics> MidPoint<D> {
    pub fn new(model: &D) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        Ok(Self {
            _phantom_data: PhantomData,
        })
    }
}

impl<D: Dynamics> Discretizer<D> for MidPoint<D> {
    fn step(
        &mut self,
        model: &D,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let x_m = state.clone() + model.dynamics(state, input) * (dt * 0.5);
        Ok(state.clone() + model.dynamics(&x_m, input) * dt)
    }
}
