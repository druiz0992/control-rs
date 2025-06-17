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
        let state_dims = D::State::dim_q() + D::State::dim_v();
        let input_dims = D::Input::dim_q();

        let params = vals.to_owned();
        let params_state = DVector::from_column_slice(&vals[0..state_dims]);
        let params_input = vals[state_dims..input_dims + state_dims].to_owned();
        let params_params = vals[input_dims + state_dims..].to_owned();

        let df_dx = (self.df_dx.0)(&params);
        let df_du = (self.df_du.0)(&params);

        let f = self
            .model
            .dynamics(&D::State::from_slice(&params_state.as_slice()), None)
            .to_vector();

        let d1 = match select {
            JacobianSelector::X => df_dx.clone(),
            JacobianSelector::U => df_du.clone(),
        };

        let compute_dk = |k_prev: &DVector<f64>, d_prev: &DMatrix<f64>, step_dt: f64| {
            let new_params_state = &params_state + step_dt * k_prev;
            let mut new_params = new_params_state.as_slice().to_vec();
            new_params.extend(&params_input);
            new_params.extend(&params_params);

            let a = (self.df_dx.0)(&new_params);
            let b = (self.df_du.0)(&new_params);
            match select {
                JacobianSelector::X => &a * d_prev * step_dt + &a,
                JacobianSelector::U => &a * d_prev * step_dt + b,
            }
        };

        let dk2 = compute_dk(&f, &d1, dt / 2.0);
        let k2 = self
            .model
            .dynamics(
                &D::State::from_slice(&(&params_state + dt / 2.0 * &f).as_slice()),
                None,
            )
            .to_vector();

        let dk3 = compute_dk(&k2, &dk2, dt / 2.0);
        let k3 = self
            .model
            .dynamics(
                &D::State::from_slice(&(&params_state + dt / 2.0 * &k2).as_slice()),
                None,
            )
            .to_vector();

        let dk4 = compute_dk(&k3, &dk3, dt);

        let jac = dt / 6.0 * (&d1 + 2.0 * dk2 + 2.0 * dk3 + dk4);

        match select {
            JacobianSelector::X => jac + DMatrix::identity(params_state.len(), params_state.len()),
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
