use crate::physics::ModelError;
use crate::physics::discretizer::{NumericDiscretizer, RK4};
use crate::physics::traits::{Discretizer, Dynamics};
use crate::utils::evaluable::{EvaluableMatrixFn, NumericFunction};
use std::sync::Arc;

pub struct RK4Numeric<D: Dynamics> {
    rk4: RK4<D>,
    df_dx: NumericFunction,
    df_du: NumericFunction,
    d2f_dxx: Option<NumericFunction>,
    d2f_dxu: Option<NumericFunction>,
    d2f_dux: Option<NumericFunction>,
    d2f_duu: Option<NumericFunction>,
}

impl<D: Dynamics> RK4Numeric<D> {
    pub fn new(
        model: &D,
        df_dx: NumericFunction,
        df_du: NumericFunction,
        d2f: Option<(
            NumericFunction,
            NumericFunction,
            NumericFunction,
            NumericFunction,
        )>,
    ) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        let rk4 = RK4::new(model)?;
        let (d2f_dxx, d2f_dxu, d2f_dux, d2f_duu) = match d2f {
            Some((dxx, dxu, dux, duu)) => (
                Some(dxx),
                Some(dxu),
                Some(dux),
                Some(duu),
            ),
            None => (None, None, None, None),
        };
        Ok(Self {
            rk4,
            df_du,
            df_dx,
            d2f_dxx,
            d2f_dxu,
            d2f_dux,
            d2f_duu,
        })
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

    fn hessian_uu(&self) -> Option<EvaluableMatrixFn> {
        self.d2f_duu
            .as_ref()
            .map(|arc| Box::new(Arc::clone(arc)) as EvaluableMatrixFn)
    }

    fn hessian_ux(&self) -> Option<EvaluableMatrixFn> {
        self.d2f_dux
            .as_ref()
            .map(|arc| Box::new(Arc::clone(arc)) as EvaluableMatrixFn)
    }

    fn hessian_xu(&self) -> Option<EvaluableMatrixFn> {
        self.d2f_dxu
            .as_ref()
            .map(|arc| Box::new(Arc::clone(arc)) as EvaluableMatrixFn)
    }

    fn hessian_xx(&self) -> Option<EvaluableMatrixFn> {
        self.d2f_dxx
            .as_ref()
            .map(|arc| Box::new(Arc::clone(arc)) as EvaluableMatrixFn)
    }
}
