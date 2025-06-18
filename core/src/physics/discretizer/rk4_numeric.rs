use crate::physics::ModelError;
use crate::physics::discretizer::{NumericDiscretizer, NumericFunction, RK4};
use crate::physics::traits::{Discretizer, Dynamics, State};
use crate::utils::evaluable::EvaluableDMatrix;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

pub struct RK4Numeric<D: Dynamics> {
    rk4: RK4<D>,
    df_dx: NumericFunction,
    df_du: NumericFunction,
    model: Arc<D>,
    dt: f64,
}

impl<D: Dynamics> RK4Numeric<D> {
    pub fn new(
        model: Arc<D>,
        df_dx: NumericFunction,
        df_du: NumericFunction,
        dt: f64,
    ) -> Result<Self, ModelError> {
        let rk4 = RK4::new(&*model)?;
        Ok(RK4Numeric {
            rk4,
            df_dx,
            df_du,
            model,
            dt,
        })
    }
    fn rk4_jacobian(&self, vals: &[f64], select: JacobianSelector) -> DMatrix<f64> {
        let dt = self.dt;
        let n = D::State::dim_q() + D::State::dim_v(); // state size
        let m = D::Input::dim_q(); // input size

        let state = DVector::from_column_slice(&vals[0..n]);
        let input = DVector::from_column_slice(&vals[n..n + m]);
        let params = &vals[n + m..];

        // Helper to combine [state, input, params]
        let build_param_vec = |s: &DVector<f64>| {
            let mut full = s.as_slice().to_vec();
            full.extend(input.as_slice());
            full.extend_from_slice(params);
            full
        };

        // Stage 1
        let f1 = self
            .model
            .dynamics(
                &D::State::from_slice(&state.as_slice()),
                Some(&D::Input::from_slice(&input.as_slice())),
            )
            .to_vector();
        let df1 = match select {
            JacobianSelector::X => (self.df_dx.0)(&build_param_vec(&state)),
            JacobianSelector::U => (self.df_du.0)(&build_param_vec(&state)),
        };

        // Stage 2
        let s2 = &state + (dt / 2.0) * &f1;
        let f2 = self
            .model
            .dynamics(
                &D::State::from_slice(&s2.as_slice()),
                Some(&D::Input::from_slice(&input.as_slice())),
            )
            .to_vector();
        let df2 = match select {
            JacobianSelector::X => (self.df_dx.0)(&build_param_vec(&s2)),
            JacobianSelector::U => (self.df_du.0)(&build_param_vec(&s2)),
        };

        // Stage 3
        let s3 = &state + (dt / 2.0) * &f2;
        let f3 = self
            .model
            .dynamics(
                &D::State::from_slice(&s3.as_slice()),
                Some(&D::Input::from_slice(&input.as_slice())),
            )
            .to_vector();
        let df3 = match select {
            JacobianSelector::X => (self.df_dx.0)(&build_param_vec(&s3)),
            JacobianSelector::U => (self.df_du.0)(&build_param_vec(&s3)),
        };

        // Stage 4
        let s4 = &state + dt * &f3;
        let _f4 = self
            .model
            .dynamics(
                &D::State::from_slice(&s4.as_slice()),
                Some(&D::Input::from_slice(&input.as_slice())),
            )
            .to_vector();
        let df4 = match select {
            JacobianSelector::X => (self.df_dx.0)(&build_param_vec(&s4)),
            JacobianSelector::U => (self.df_du.0)(&build_param_vec(&s4)),
        };

        // RK4 Jacobian formula
        let jac = dt / 6.0 * (&df1 + 2.0 * &df2 + 2.0 * &df3 + &df4);
        //let jac = dt / 6.0 * (&df2);

        match select {
            JacobianSelector::X => jac + DMatrix::identity(n, n),
            JacobianSelector::U => jac,
        }
    }

    pub fn jacobian_x(&self, vals: &[f64]) -> DMatrix<f64> {
        self.rk4_jacobian(vals, JacobianSelector::X)
    }

    pub fn jacobian_u(&self, vals: &[f64]) -> DMatrix<f64> {
        self.rk4_jacobian(vals, JacobianSelector::U)
    }
}

#[derive(Copy, Clone)]
enum JacobianSelector {
    X,
    U,
}

impl<D: Dynamics + 'static + Send + Sync> RK4Numeric<D> {
    fn clone_for_eval(&self) -> Self {
        Self {
            rk4: self.rk4.clone(), // make sure RK4 is clonable
            model: Arc::clone(&self.model),
            df_dx: self.df_dx.clone(),
            df_du: self.df_du.clone(),
            dt: self.dt,
        }
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

impl<D: Dynamics + 'static + Send + Sync> NumericDiscretizer<D> for RK4Numeric<D> {
    fn to_numeric_df_dx(&self) -> EvaluableDMatrix {
        let this = Arc::new(self.clone_for_eval());
        let numeric_function = move |vals: &[f64]| this.jacobian_x(vals);
        Box::new(NumericFunction(Arc::new(numeric_function)))
    }

    fn to_numeric_df_du(&self) -> EvaluableDMatrix {
        let this = Arc::new(self.clone_for_eval());
        let numeric_function = move |vals: &[f64]| this.jacobian_u(vals);
        Box::new(NumericFunction(Arc::new(numeric_function)))
    }
}
