use crate::numeric_services::symbolic::{
    ExprRegistry, ExprScalar, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics, State};
use std::sync::Arc;

#[derive(Default)]
pub struct RK4;

impl RK4 {
    pub fn new() -> Self {
        Self
    }
}

impl<D> Discretizer<D> for RK4
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
        let k1 = model.dynamics(state, input);
        let k2 = model.dynamics(&(state.clone() + k1.clone() * (dt * 0.5)), input);
        let k3 = model.dynamics(&(state.clone() + k2.clone() * (dt * 0.5)), input);
        let k4 = model.dynamics(&(state.clone() + k3.clone() * dt), input);

        Ok(state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0))
    }
}

impl Describable for RK4 {
    fn name(&self) -> &'static str {
        "RK4"
    }
}
pub struct RK4Symbolic {
    step_func: SymbolicFunction,
    registry: Arc<ExprRegistry>,
}

impl RK4Symbolic {
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let state = registry.get_vector("state")?;

        let dt = ExprScalar::new("dt");
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
        let mut result_s = k1_s
            .add(&k4_s)
            .add(&k2_s.scalef(2.0))
            .add(&k3_s.scalef(2.0))
            .wrap();
        result_s = result_s.scale(&dt6);
        result_s = result_s.add(&state);
        let step_func = result_s.to_fn(&registry)?;

        // if model allows inputs, redefine vars to include inputs
        let mut vars = state.as_vec();
        if let Ok(input) = registry.get_vector("input") {
            vars.extend_from_slice(&input.as_vec());
        }
        let step_func = SymbolicFunction::new(step_func, &vars);

        Ok(Self {
            step_func,
            registry,
        })
    }
}

impl<D> Discretizer<D> for RK4Symbolic
where
    D: Dynamics,
{
    fn step(
        &mut self,
        _model: &D,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        self.registry.insert_var("dt", dt);

        let mut vals = state.as_vec();
        if let Some(u) = input {
            vals.extend_from_slice(u);
        }

        Ok(self.step_func.eval(&vals).try_into_eval_result()?)
    }
}

impl Describable for RK4Symbolic {
    fn name(&self) -> &'static str {
        "RK4-Symbolic"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::models::{DoublePendulum, DoublePendulumState};
    use crate::utils::within_tolerance;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_rk4_step() {
        let mut rk4 = RK4::new();
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);
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
        let mut rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
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
            let mut rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
            let mut rk4 = RK4::new();
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
