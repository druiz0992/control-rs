use super::model::CartPole;
use super::state::CartPoleState;
use crate::common::Labelizable;
use crate::numeric_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::traits::Dynamics;
use crate::physics::{constants as c, energy::Energy};
use std::sync::Arc;

impl Dynamics for CartPole {
    type State = CartPoleState;

    fn dynamics(&self, state: &Self::State, input: Option<&[f64]>) -> Self::State {
        let [m_p, m_c, friction_coeff, air_resistance_coeff, l] = self.extract(&[
            "pole_mass",
            "cart_mass",
            "friction_coeff",
            "air_resistance_coeff",
            "l",
        ]);
        let [v_x, theta, omega] = state.extract(&["v_x", "theta", "omega"]);
        let u = input.unwrap_or(&[0.0])[0];

        // damping
        let cart_friction = -friction_coeff * v_x;
        let pendulum_damping = -air_resistance_coeff * omega * omega.abs();

        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let total_mass = m_p + m_c;

        let denom = l * (4.0 / 3.0 - m_p * cos_theta * cos_theta / total_mass);

        let domega = (c::GRAVITY * sin_theta
            + cos_theta * (-u - cart_friction - m_p * l * omega * omega * sin_theta) / total_mass
            + pendulum_damping / (m_p + l))
            / denom;

        let dv = (u + cart_friction + m_p * l * (omega * omega * sin_theta - domega * cos_theta))
            / total_mass;

        CartPoleState {
            pos_x: v_x,
            v_x: dv,
            theta: omega,
            omega: domega,
        }
    }

    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector {
        // Define symbolic variables
        let v_x = state.get(CartPoleState::index_of("v_x")).unwrap();
        let theta = state.get(CartPoleState::index_of("theta")).unwrap();
        let omega = state.get(CartPoleState::index_of("omega")).unwrap();
        let u = registry.get_scalar(c::INPUT_SYMBOLIC).unwrap();

        let m_p = registry.get_scalar(c::MASS_POLE_SYMBOLIC).unwrap();
        let m_c = registry.get_scalar(c::MASS_CART_SYMBOLIC).unwrap();
        let friction_coeff = registry.get_scalar(c::FRICTION_COEFF_SYMBOLIC).unwrap();
        let air_resistance_coeff = registry
            .get_scalar(c::AIR_RESISTANCE_COEFF_SYMBOLIC)
            .unwrap();
        let l = registry.get_scalar(c::LENGTH_SYMBOLIC).unwrap();
        let g = registry.get_scalar(c::GRAVITY_SYMBOLIC).unwrap();

        // damping
        let cart_friction = friction_coeff.scalef(-1.0).mul(&v_x);
        let pendulum_damping = air_resistance_coeff
            .scalef(-1.0)
            .mul(&omega)
            .mul(&omega.abs());

        // Common terms
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let total_mass = m_p.add(&m_c).wrap();
        let scaling = 4.0 / 3.0;

        let denom = l
            .scalef(scaling)
            .sub(&l.mul(&m_p).mul(&cos_theta).mul(&cos_theta).div(&total_mass))
            .wrap();

        let domega = g
            .mul(&sin_theta)
            .add(&cos_theta)
            .mul(
                &u.scalef(-1.0)
                    .add(&cart_friction.scalef(-1.0).wrap())
                    .sub(&m_p)
                    .mul(&l)
                    .mul(&omega)
                    .mul(&omega)
                    .mul(&sin_theta)
                    .wrap(),
            )
            .div(&total_mass)
            .add(&pendulum_damping)
            .div(&m_p.add(&l).wrap())
            .wrap()
            .div(&denom)
            .wrap();

        let dv = u
            .add(&cart_friction)
            .add(&m_p)
            .mul(&l)
            .mul(
                &omega
                    .mul(&omega)
                    .mul(&sin_theta)
                    .sub(&domega)
                    .mul(&cos_theta)
                    .wrap(),
            )
            .wrap()
            .div(&total_mass);
        ExprVector::from_vec(vec![v_x, dv, omega, domega])
    }

    fn energy(&self, state: &Self::State) -> Energy {
        let [m_p, m_c, l] = self.extract(&["pole_mass", "cart_mass", "l"]);
        let [v_x, theta, omega] = state.extract(&["v_x", "theta", "omega"]);

        let omega_square = omega.powi(2);
        let vx_square = v_x.powi(2);

        let kintetic_cart = 0.5 * m_c * vx_square;
        let kinetic_pole = 0.5
            * m_p
            * ((v_x + l / 2.0 * theta.cos() * omega_square).powi(2)
                + (l / 2.0 * theta.sin() * omega_square).powi(2));

        let kinetic = kinetic_pole + kintetic_cart;
        let potential = 0.5 * m_p * c::GRAVITY * l * theta.cos();

        Energy::new(kinetic, potential)
    }
}

#[cfg(test)]
mod tests {
    use crate::numeric_services::symbolic::{SymbolicExpr, TryIntoEvalResult};
    use crate::utils::within_tolerance;

    use super::*;
    use proptest::prelude::*;
    use std::f64::consts::PI;

    #[test]
    fn test_dynamics() {
        let cart_pole = CartPole::new(1.0, 2.0, 0.0, 0.0, 1.0, None);
        let state = CartPoleState::new(0.0, 0.0, 0.0, 0.0);

        let new_state = cart_pole.dynamics(&state, None);

        assert!(new_state.pos_x.abs() < 1e-6);
        assert!(new_state.v_x.abs() < 1e-6);
        assert!(new_state.theta.abs() < 1e-6);
        assert!(new_state.omega.abs() < 1e-6);
    }

    #[test]
    #[ignore]
    fn test_dynamics_symbolic() {
        let registry = Arc::new(ExprRegistry::new());
        let cart_pole = CartPole::new(1.0, 2.0, 0.0, 0.0, 1.0, Some(&registry));
        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let dynamics_func = cart_pole.dynamics_symbolic(&state_symbol, &registry);
        dbg!(&dynamics_func);
    }

    proptest! {
        #[test]
        fn test_symbolic_dynamics_randomized(
            pos_x in -5.0..5.0,
            v_x in -5.0..5.0,
            theta in 0.0..(2.0 * PI),
            omega in -5.0..5.0,
            pole_mass in 0.1f64..10.0,
            cart_mass in 0.1f64..10.0,
            friction_coeff in 0.0f64..5.0,
            air_resistance_coeff in 0.0f64..5.0,
            l in 0.1f64..5.0,
        ) {
            let registry = Arc::new(ExprRegistry::new());
            registry.insert_var("pos_x", pos_x);
            registry.insert_var("v_x", v_x);
            registry.insert_var("theta", theta);
            registry.insert_var("omega", omega);

            let cart_pole = CartPole::new(pole_mass, cart_mass, friction_coeff, air_resistance_coeff, l, Some(&registry));

            let state = CartPoleState::new(pos_x, v_x, theta, omega);
            let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();

            let new_state = cart_pole.dynamics(&state, None);
            let dynamics_func = cart_pole
                .dynamics_symbolic(&state_symbol, &registry)
                .to_fn(&registry)
                .unwrap();
            let new_state_symbol: CartPoleState = dynamics_func(None).try_into_eval_result().unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-6; // tolerance for floating point comparison

            assert!(within_tolerance(new_state.pos_x, new_state_symbol.pos_x, tol), "pos_x mismatch");
            assert!(within_tolerance(new_state.v_x, new_state_symbol.v_x, tol), "v_x mismatch");
            assert!(within_tolerance(new_state.theta, new_state_symbol.theta, tol), "theta mismatch");
            assert!(within_tolerance(new_state.omega, new_state_symbol.omega, tol), "omega mismatch");
        }
    }
}
