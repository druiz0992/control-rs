use crate::numeric_services::symbolic::{
    ExprRegistry, ExprScalar, ExprVector, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use crate::physics::discretizer::NumericFunction;
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::traits::{Discretizer, Dynamics, State};
use crate::physics::{ModelError, constants as c};
use crate::utils::evaluable::Evaluable;
use nalgebra::{DMatrix, DVector};
use std::marker::PhantomData;
use std::sync::Arc;

use super::SymbolicDiscretizer;

#[derive(Default, Clone)]
pub struct RK4<D: Dynamics> {
    _phantom_data: PhantomData<D>,
}

impl<D: Dynamics> RK4<D> {
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

impl<D: Dynamics> Discretizer<D> for RK4<D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let k1 = model.dynamics(state, input);
        let k2 = model.dynamics(&(state.clone() + k1.clone() * (dt * 0.5)), input);
        let k3 = model.dynamics(&(state.clone() + k2.clone() * (dt * 0.5)), input);
        let k4 = model.dynamics(&(state.clone() + k3.clone() * dt), input);

        Ok(state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0))
    }
}

pub struct RK4Symbolic<D: SymbolicDynamics> {
    step_func: SymbolicFunction,
    registry: Arc<ExprRegistry>,
    dynamics: ExprVector,
    _phantom_data: PhantomData<D>,
}

impl<D: SymbolicDynamics> RK4Symbolic<D> {
    pub fn new(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }

        let state = registry.get_vector(c::STATE_SYMBOLIC)?;

        let dt = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let dt2 = dt.scalef(0.5).wrap();
        let dt6 = dt.scalef(1.0 / 6.0).wrap();

        let k1_s = model.dynamics_symbolic(&state, &registry).wrap();
        let k2_s = model
            .dynamics_symbolic(&k1_s.scale(&dt2).add(&state).wrap(), &registry)
            .wrap();
        let k3_s = model
            .dynamics_symbolic(&k2_s.scale(&dt2).add(&state).wrap(), &registry)
            .wrap();
        let k4_s = model
            .dynamics_symbolic(&k3_s.scale(&dt).add(&state).wrap(), &registry)
            .wrap();
        let mut dynamics = k1_s
            .add(&k4_s)
            .add(&k2_s.scalef(2.0))
            .add(&k3_s.scalef(2.0))
            .wrap();
        dynamics = dynamics.scale(&dt6);
        dynamics = dynamics.add(&state).wrap();
        let step_func = dynamics.to_fn(&registry)?;

        // if model allows inputs, redefine vars to include inputs
        let mut vars = state.to_vec();
        if let Ok(input) = registry.get_vector(c::INPUT_SYMBOLIC) {
            vars.extend_from_slice(&input.to_vec());
        }
        let step_func = SymbolicFunction::new(step_func, &vars);

        Ok(Self {
            step_func,
            registry,
            dynamics,
            _phantom_data: PhantomData,
        })
    }
}

impl<D: SymbolicDynamics> Discretizer<D> for RK4Symbolic<D> {
    fn step(
        &self,
        _model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        self.registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

        let mut vals = state.to_vec();
        if let Some(u) = input {
            vals.extend_from_slice(&u.to_vec());
        }

        Ok(self.step_func.eval(&vals).try_into_eval_result()?)
    }
}

impl<D: SymbolicDynamics> SymbolicDiscretizer<D> for RK4Symbolic<D> {
    fn jacobian_x(&self) -> Result<SymbolicFunction, ModelError> {
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol);

        let jacobian_x = self
            .dynamics
            .jacobian(&state_symbol)?
            .to_fn(&self.registry)?;
        Ok(SymbolicFunction::new(jacobian_x, &jacobian_symbols))
    }

    fn jacobian_u(&self) -> Result<SymbolicFunction, ModelError> {
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol);

        let jacobian_u = self
            .dynamics
            .jacobian(&input_symbol)?
            .to_fn(&self.registry)?;
        Ok(SymbolicFunction::new(jacobian_u, &jacobian_symbols))
    }
}

pub struct RK4Numeric<'a, D: Dynamics> {
    discretizer: RK4<D>,
    df_dx: NumericFunction,
    df_du: NumericFunction,
    model: &'a D,
    dt: f64,
}

impl<'a, D: Dynamics> RK4Numeric<'a, D> {
    pub fn new(
        model: &'a D,
        df_dx: NumericFunction,
        df_du: NumericFunction,
        dt: f64,
    ) -> Result<Self, ModelError> {
        let rk4 = RK4::new(model)?;
        Ok(RK4Numeric {
            discretizer: rk4,
            df_dx,
            df_du,
            model,
            dt,
        })
    }

    pub fn jacobians(&self, vals: &[f64]) -> (DMatrix<f64>, DMatrix<f64>) {
        let dt = self.dt;
        let state_dims = D::State::dim_q() + D::State::dim_v();
        let mut params = vals.to_owned();
        let params_state = DVector::from_column_slice(&vals[0..state_dims]);
        let params_input = vals[state_dims..].to_owned();
        //let params_input = input.map_or(D::Input::default().to_vec(), |i| i.to_vec());

        let df_dx = (self.df_dx)(&params);
        let df_du = (self.df_du)(&params);
        let f = self
            .model
            .dynamics(&D::State::from_slice(&params_state.as_slice()), None)
            .to_vec();

        let k1 = DVector::from_vec(f.clone());
        let dk1_dx = &df_dx;
        let dk1_du = &df_du;

        let new_params_state = &params_state + dt / 2.0 * &k1;
        params = new_params_state.as_slice().to_vec();
        params.extend(params_input.as_slice());
        let k2 = self
            .model
            .dynamics(&D::State::from_slice(new_params_state.as_slice()), None)
            .to_vector();
        let a_k2 = (self.df_dx)(&params);
        let b_k2 = (self.df_du)(&params);

        let tmp_dx = &a_k2 * dk1_dx * dt / 2.0;
        let dk2_dx = &tmp_dx + &a_k2;
        let tmp_du = &a_k2 * dk1_du * dt / 2.0;
        let dk2_du = &tmp_du + b_k2;

        let new_params_state = &params_state + dt / 2.0 * &k2;
        params = new_params_state.as_slice().to_vec();
        params.extend(params_input.as_slice());
        let k3 = self
            .model
            .dynamics(&D::State::from_slice(new_params_state.as_slice()), None)
            .to_vector();
        let a_k3 = (self.df_dx)(&params);
        let b_k3 = (self.df_du)(&params);
        let tmp_dx = &a_k3 * &dk2_dx * dt / 2.0;
        let dk3_dx = &tmp_dx + &a_k3;
        let tmp_du = &a_k3 * &dk2_du * dt / 2.0;
        let dk3_du = &tmp_du + b_k3;

        let new_params_state = &params_state + dt * &k3;
        params = new_params_state.as_slice().to_vec();
        params.extend(params_input.as_slice());
        //let k4 = DVector::from_vec(eval_f(&params));
        let a_k4 = (self.df_dx)(&params);
        let b_k4 = (self.df_du)(&params);
        let tmp_dx = &a_k4 * &dk3_dx * dt;
        let dk4_dx = &tmp_dx + &a_k4;
        let tmp_du = &a_k4 * &dk3_du * dt;
        let dk4_du = &tmp_du + b_k4;

        let dx_next_dx = dt / 6.0 * (dk1_dx + 2.0 * dk2_dx + 2.0 * dk3_dx + dk4_dx)
            + DMatrix::identity(params_state.len(), params_state.len());
        let dx_next_du = dt / 6.0 * (dk1_du + 2.0 * dk2_du + 2.0 * dk3_du + dk4_du);

        (dx_next_dx, dx_next_du)
    }
}

impl<'a, D: Dynamics> Discretizer<D> for RK4Numeric<'a, D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        self.discretizer.step(model, state, input, dt)
    }
}

impl<'a, D: Dynamics> Evaluable for RK4Numeric<'a, D> {
    type Output = (DMatrix<f64>, DMatrix<f64>);

    fn evaluate(&self, vals: &[f64]) -> Result<Self::Output, ModelError> {
        Ok(self.jacobians(vals))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::models::{DoublePendulum, DoublePendulumState};
    use crate::utils::helpers::within_tolerance;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_rk4_step() {
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);
        let rk4 = RK4::new(&dynamics).unwrap();
        let initial_state = DoublePendulumState::new(0.0, 0.0, 0.0, 0.0);
        let dt = 0.1;

        let next_state = rk4.step(&dynamics, &initial_state, None, dt).unwrap();

        assert!((next_state.omega1 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_rk4_symbolic_step() {
        let registry = Arc::new(ExprRegistry::new());
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, Some(&registry));

        let theta1 = 0.0;
        let omega1 = 0.0;
        let theta2 = 0.0;
        let omega2 = 0.0;

        let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
        let rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
        let dt = 0.1;

        let next_state = rk4_symbolic.step(&dynamics, &state, None, dt).unwrap();

        assert!((next_state.omega1 - 0.0).abs() < 1e-6);
    }

    proptest! {
        #[test]
        fn test_symbolic_equivalence(
            theta1 in 0.0..(2.0 * PI),
            theta2 in 0.0..(2.0 * PI),
            omega1 in -5.0..5.0,
            omega2 in -5.0..5.0,
            m1 in 0.1f64..10.0,
            m2 in 0.1f64..10.0,
            l1 in 0.1f64..5.0,
            l2 in 0.1f64..5.0,
            air_resistance_coeff in 0.0f64..5.0
        ) {
            let registry = Arc::new(ExprRegistry::new());
            let dynamics = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
            let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
            let rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
            let rk4 = RK4::new(&dynamics).unwrap();
            let dt = 0.1;


            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            let next_state_symbolic = rk4_symbolic.step(&dynamics, &state, None, dt).unwrap();
            let next_state = rk4.step(&dynamics, &state, None, dt).unwrap();

            assert!(within_tolerance(next_state.theta1, next_state_symbolic.theta1, tol), "theta1 mismatch");
            assert!(within_tolerance(next_state.omega1, next_state_symbolic.omega1, tol), "omega1 mismatch");
            assert!(within_tolerance(next_state.theta2, next_state_symbolic.theta2, tol), "theta2 mismatch");
            assert!(within_tolerance(next_state.omega2, next_state_symbolic.omega2, tol), "omega2 mismatch");
        }
    }
}
