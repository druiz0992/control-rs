use super::input::Quadrotor2DInput;
use super::model::Quadrotor2D;
use super::state::Quadrotor2DState;
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::traits::{Dynamics, State};
use crate::physics::{constants as c, energy::Energy};
use crate::utils::Labelizable;
use std::sync::Arc;
use symbolic_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};

impl Dynamics for Quadrotor2D {
    type State = Quadrotor2DState;
    type Input = Quadrotor2DInput;

    fn dynamics(&self, s: &Self::State, input: Option<&Self::Input>) -> Quadrotor2DState {
        let [m, j, l] = self.extract(&["m", "j", "l"]);
        let [theta, v_x, v_y, omega] = s.extract(&["theta", "v_x", "v_y", "omega"]);
        let input = input.unwrap_or(&Self::Input::default()).clone();
        let [u1, u2] = input.extract(&["u1", "u2"]);

        let g = c::GRAVITY;

        let scaled_m = 1.0 / m * (u1 + u2);
        let d_vx = scaled_m * theta.sin();
        let d_vy = scaled_m * theta.cos() - g;
        let d_omega = l / (2.0 * j) * (u2 - u1);

        Quadrotor2DState {
            pos_x: v_x,
            pos_y: v_y,
            theta: omega,
            v_x: d_vx,
            v_y: d_vy,
            omega: d_omega,
        }
    }

    fn energy(&self, s: &Quadrotor2DState) -> Option<Energy> {
        let [m, j] = self.extract(&["m", "j"]);
        let [pos_y, v_x, v_y, omega] = s.extract(&["pos_y", "v_x", "v_y", "omega"]);

        let kinetic = 0.5 * m * (v_x * v_x + v_y * v_y) + 0.5 * j * omega * omega;
        let potential = m * c::GRAVITY * pos_y;

        Some(Energy::new(kinetic, potential))
    }

    fn state_dims(&self) -> (usize, usize) {
        (Quadrotor2DState::dim_q(), Quadrotor2DState::dim_v())
    }
}

impl SymbolicDynamics for Quadrotor2D {
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector {
        // Define symbolic variables
        let theta = state.get(Quadrotor2DState::index_of("theta")).unwrap();
        let v_x = state.get(Quadrotor2DState::index_of("v_x")).unwrap();
        let v_y = state.get(Quadrotor2DState::index_of("v_y")).unwrap();
        let omega = state.get(Quadrotor2DState::index_of("omega")).unwrap();

        let u = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();

        let m = registry.get_scalar("m").unwrap_or(ExprScalar::new("m"));
        let j = registry.get_scalar("j").unwrap_or(ExprScalar::new("j"));
        let l = registry.get_scalar("l").unwrap_or(ExprScalar::new("l"));
        let g = registry.get_scalar(c::GRAVITY_SYMBOLIC).unwrap();

        // Common terms
        let scaled_m = u[0].add(&u[1]).wrap().div(&m).wrap();
        let d_vx = scaled_m.mul(&theta.sin()).wrap();
        let d_vy = scaled_m.mul(&theta.cos()).sub(&g).wrap();
        let d_omega = l
            .div(&j.scalef(2.0).wrap())
            .wrap()
            .mul(&u[1].sub(&u[0]).wrap())
            .wrap();

        // Return as a symbolic vector
        ExprVector::from_vec(vec![v_x, v_y, omega, d_vx, d_vy, d_omega])
    }
}

#[cfg(test)]
mod tests {
    use general::helpers::within_tolerance;
    use symbolic_services::symbolic::{SymbolicExpr, TryIntoEvalResult};

    use crate::physics::models::state::SymbolicResult;

    use super::*;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_dynamics() {
        let quad = Quadrotor2D::new(1.0, 2.0, 1.0, None);
        let state = Quadrotor2DState::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        let new_state = quad.dynamics(&state, None);

        assert!(new_state.pos_x.abs() < 1e-6);
        assert!(new_state.pos_y.abs() < 1e-6);
        assert!(new_state.theta.abs() < 1e-6);
        assert!(new_state.v_x.abs() < 1e-6);
        assert!((new_state.v_y + c::GRAVITY).abs() < 1e-6);
        assert!(new_state.omega.abs() < 1e-6);
    }

    #[test]
    #[ignore]
    fn test_dynamics_symbolic() {
        let registry = Arc::new(ExprRegistry::new());
        let quad = Quadrotor2D::new(1.0, 2.0, 1.0, Some(&registry));
        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let dynamics_func = quad.dynamics_symbolic(&state_symbol, &registry);
        dbg!(&dynamics_func);
    }

    proptest! {
        #[test]
        fn test_symbolic_dynamics_randomized(
            pos_x in -5.0..5.0,
            pos_y in -5.0..5.0,
            theta in 0.0..(2.0 * PI),
            v_x in -5.0..5.0,
            v_y in -5.0..5.0,
            omega in -5.0..5.0,
            m in 0.1f64..10.0,
            j in 0.1f64..10.0,
            l in 0.1f64..5.0,
            u1 in -5.0..5.0,
            u2 in -5.0..5.0,
        ) {
            let registry = Arc::new(ExprRegistry::new());
            registry.insert_var("pos_x", pos_x);
            registry.insert_var("pos_y", pos_y);
            registry.insert_var("theta", theta);
            registry.insert_var("v_x", v_x);
            registry.insert_var("v_y", v_y);
            registry.insert_var("omega", omega);
            registry.insert_var("m",m);
            registry.insert_var("j",j);
            registry.insert_var("l",l);
            registry.insert_var("u1",u1);
            registry.insert_var("u2",u2);

            let quad = Quadrotor2D::new(m, j, l, Some(&registry));

            let state = Quadrotor2DState::new(pos_x, pos_y, theta, v_x, v_y, omega);
            let input = Quadrotor2DInput::new(u1, u2);
            let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();

            let new_state = quad.dynamics(&state, Some(&input));
            let dynamics_func =quad
                .dynamics_symbolic(&state_symbol, &registry)
                .to_fn(&registry)
                .unwrap();

            let new_state_symbol: Quadrotor2DState = SymbolicResult::new(dynamics_func(None)).try_into_eval_result().unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            assert!(within_tolerance(new_state.pos_x, new_state_symbol.pos_x, tol), "pos_x mismatch");
            assert!(within_tolerance(new_state.pos_y, new_state_symbol.pos_y, tol), "pos_y mismatch");
            assert!(within_tolerance(new_state.theta, new_state_symbol.theta, tol), "theta mismatch");
            assert!(within_tolerance(new_state.v_x, new_state_symbol.v_x, tol), "v_x mismatch");
            assert!(within_tolerance(new_state.v_y, new_state_symbol.v_y, tol), "v_y mismatch");
            assert!(within_tolerance(new_state.omega, new_state_symbol.omega, tol), "omega mismatch");
        }
    }
}
