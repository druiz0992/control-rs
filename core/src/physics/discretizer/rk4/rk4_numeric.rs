use crate::physics::ModelError;
use crate::physics::discretizer::{NumericDiscretizer, RK4};
use crate::physics::traits::{Discretizer, Dynamics};
use crate::utils::evaluable::{EvaluableMatrixFn, NumericFunction};
use std::sync::Arc;

pub struct RK4Numeric<D: Dynamics> {
    rk4: RK4<D>,
    df_dx: NumericFunction,
    df_du: NumericFunction,
}

impl<D: Dynamics> RK4Numeric<D> {
    pub fn new(
        model: &D,
        df_dx: NumericFunction,
        df_du: NumericFunction,
    ) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        let rk4 = RK4::new(model)?;
        Ok(Self { rk4, df_du, df_dx })
    }
}

impl<D: Dynamics> Discretizer<D> for RK4Numeric<D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        self.rk4.step(model, state, input, dt)
    }
}

impl<D: Dynamics> NumericDiscretizer<D> for RK4Numeric<D> {
    fn jacobian_u(&self) -> EvaluableMatrixFn {
        Box::new(Arc::clone(&self.df_du))
    }

    fn jacobian_x(&self) -> EvaluableMatrixFn {
        Box::new(Arc::clone(&self.df_dx))
    }
}
