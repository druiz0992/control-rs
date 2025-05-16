use super::model::BouncingBall;
use super::state::BouncingBallState;
use crate::common::Labelizable;
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::traits::State;
use crate::physics::{Energy, constants as c, traits::Dynamics};
use std::sync::Arc;

/// M * v_dot  + M * g = J' * u + f_friction, where M = mass * identity(2), g = [0, 9.81], J = [0,1]
/// mu is normal force (scalar), v is velocity (2D vector), and q is position (2D vector),
///  f_friction = [ -mu * friction_coeff * sign(v_x), 0]
impl Dynamics for BouncingBall {
    type State = BouncingBallState;

    fn dynamics(&self, state: &Self::State, _input: Option<&[f64]>) -> Self::State {
        let [m, friction_coeff] = self.extract(&["m", "friction_coeff"]);
        let [pos_y, v_x, v_y] = state.extract(&["pos_y", "v_x", "v_y"]);
        let g = c::GRAVITY;

        // friction
        let mut f_x = 0.0;
        let mut f_y = -m * g;

        let in_contact = pos_y <= 0.0 && v_y <= 0.0;

        if in_contact {
            let normal_force = m * g;
            f_y += normal_force;

            let friction_force = -friction_coeff * normal_force * v_x.signum();
            f_x += friction_force;
        }

        let a_x = f_x / m;
        let a_y = f_y / m;

        BouncingBallState {
            pos_x: v_x,
            pos_y: v_y,
            v_x: a_x,
            v_y: a_y,
        }
    }

    fn state_dims(&self) -> (usize, usize) {
        (BouncingBallState::dim_q(), BouncingBallState::dim_v())
    }

    fn energy(&self, state: &Self::State) -> Option<Energy> {
        let [m] = self.extract(&["m"]);
        let [pos_y, v_x, v_y] = state.extract(&["pos_y", "v_x", "v_y"]);

        let kinetic = 0.5 * m * (v_x.powi(2) + v_y.powi(2));
        let potential = m * c::GRAVITY * pos_y;

        Some(Energy::new(kinetic, potential))
    }
}

impl SymbolicDynamics for BouncingBall {
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector {
        let pos_y = state.get(BouncingBallState::index_of("pos_y")).unwrap();
        let v_x = state.get(BouncingBallState::index_of("v_x")).unwrap();
        let v_y = state.get(BouncingBallState::index_of("v_y")).unwrap();

        let friction_coeff = registry.get_scalar(c::FRICTION_COEFF_SYMBOLIC).unwrap();
        let m = registry.get_scalar(c::MASS_SYMBOLIC).unwrap();
        let g = registry.get_scalar(c::GRAVITY_SYMBOLIC).unwrap();
        let zero = ExprScalar::zero();
        let normal_force = m.mul(&g).scalef(1.0);

        let f_x_opt2 = zero.clone();
        let f_y_opt2 = normal_force.scalef(-1.0);

        let in_contact = pos_y.lte(&zero).and(&v_y.lte(&zero));

        let f_x_opt1 = friction_coeff
            .scalef(-1.0)
            .mul(&normal_force)
            .mul(&v_x.sign());
        let f_y_opt1 = zero;

        // opt1 if condition true
        let a_x = in_contact.select(&f_x_opt1, &f_x_opt2);
        let a_y = in_contact.select(&f_y_opt1, &f_y_opt2);

        ExprVector::from_vec(vec![v_x, v_y, a_x.div(&m), a_y.div(&m)])
    }

    /// linear term = M * (dt * g - v_current)
    fn cost_linear_term(
        &self,
        dt: &ExprScalar,
        registry: &Arc<ExprRegistry>,
    ) -> Option<ExprVector> {
        let mass_matrix = registry.get_matrix(c::MASS_MATRIX_SYMBOLIC).unwrap();
        let g = registry.get_scalar(c::GRAVITY_SYMBOLIC).unwrap();
        let g_vector = ExprVector::new(&["0", "1"]).scale(&g);
        let v = registry.get_vector(c::STATE_V_SYMBOLIC).unwrap();

        Some(
            mass_matrix
                .matmul_vec(&g_vector.scale(dt).sub(&v).wrap())
                .wrap(),
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::numeric_services::symbolic::{SymbolicExpr, TryIntoEvalResult};
    use crate::utils::within_tolerance;

    use super::*;
    use proptest::prelude::*;

    #[test]
    fn test_dynamics() {
        let bouncing_ball = BouncingBall::new(1.0, 0.0, None);
        let state = BouncingBallState::new(0.0, 0.0, 0.0, 0.0);

        let new_state = bouncing_ball.dynamics(&state, None);

        assert!(new_state.pos_x.abs() < 1e-6);
        assert!(new_state.pos_y.abs() < 1e-6);
        assert!(new_state.v_x.abs() < 1e-6);
        assert!(new_state.v_y.abs() < 1e-6);
    }

    #[test]
    #[ignore]
    fn test_dynamics_symbolic() {
        let registry = Arc::new(ExprRegistry::new());
        let bouncing_ball = BouncingBall::new(1.0, 2.0, Some(&registry));
        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let dynamics_func = bouncing_ball.dynamics_symbolic(&state_symbol, &registry);
        dbg!(&dynamics_func);
    }

    proptest! {
        #[test]
        fn test_symbolic_dynamics_randomized(
            pos_x in -5.0..5.0,
            pos_y in -5.0..5.0,
            v_x in -5.0..5.0,
            v_y in -5.0..5.0,
            m in 0.1f64..10.0,
            friction_coeff in 0.0f64..5.0,
        ) {
            let registry = Arc::new(ExprRegistry::new());
            registry.insert_var("pos_x", pos_x);
            registry.insert_var("pos_y", pos_y);
            registry.insert_var("v_x", v_x);
            registry.insert_var("v_y", v_y);

            let bouncing_ball = BouncingBall::new(m, friction_coeff, Some(&registry));
            let state = BouncingBallState::new(pos_x, pos_y, v_x, v_y);

            let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();

            let new_state = bouncing_ball.dynamics(&state, None);
            let dynamics_func = bouncing_ball
                .dynamics_symbolic(&state_symbol, &registry)
                .to_fn(&registry)
                .unwrap();
            let new_state_symbol: BouncingBallState = dynamics_func(None).try_into_eval_result().unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-3; // tolerance for floating point comparison

            /*
            if !within_tolerance(new_state.v_x, new_state_symbol.v_x, tol) ||
               !within_tolerance(new_state.v_y, new_state_symbol.v_y, tol) {
                dbg!(&new_state, &new_state_symbol);
                dbg!(pos_x, pos_y, v_x, v_y,m,friction_coeff);
               }
            */

            assert!(within_tolerance(new_state.pos_x, new_state_symbol.pos_x, tol), "pos_x mismatch");
            assert!(within_tolerance(new_state.pos_y, new_state_symbol.pos_y, tol), "pos_y mismatch");
            assert!(within_tolerance(new_state.v_x, new_state_symbol.v_x, tol), "v_x mismatch");
            assert!(within_tolerance(new_state.v_y, new_state_symbol.v_y, tol), "v_y mismatch");
        }
    }
}
