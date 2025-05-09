use super::state::CartPoleState;
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::constants as c;
use crate::physics::traits::State;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CartPole {
    pole_mass: f64,
    cart_mass: f64,
    l: f64,
    friction_coeff: f64,
    air_resistance_coeff: f64,
}

impl CartPole {
    pub fn new(
        pole_mass: f64,
        cart_mass: f64,
        friction_coeff: f64,
        air_resistance_coeff: f64,
        l: f64,
        registry: Option<&Arc<ExprRegistry>>,
    ) -> Self {
        if let Some(registry) = registry {
            registry.insert_scalar(c::MASS_POLE_SYMBOLIC, pole_mass);
            registry.insert_scalar(c::MASS_CART_SYMBOLIC, cart_mass);
            registry.insert_scalar(c::FRICTION_COEFF_SYMBOLIC, friction_coeff);
            registry.insert_scalar(c::AIR_RESISTANCE_COEFF_SYMBOLIC, air_resistance_coeff);
            registry.insert_scalar(c::LENGTH_SYMBOLIC, l);
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);

            registry.insert_vector(c::STATE_SYMBOLIC, CartPoleState::labels());
            registry.insert_scalar_expr(c::INPUT_SYMBOLIC, ExprScalar::new("u1"));
            registry.insert_var("u1", 0.0);
        }

        CartPole {
            pole_mass,
            cart_mass,
            friction_coeff,
            air_resistance_coeff,
            l,
        }
    }

    pub fn parameters(&self) -> (f64, f64, f64, f64, f64) {
        (
            self.pole_mass,
            self.cart_mass,
            self.friction_coeff,
            self.air_resistance_coeff,
            self.l,
        )
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cart_pole_creation_without_registry() {
        let pole_mass = 1.0;
        let cart_mass = 2.0;
        let friction_coeff = 0.0;
        let air_resistance_coeff = 0.0;
        let l = 0.5;

        let cart_pole = CartPole::new(
            pole_mass,
            cart_mass,
            friction_coeff,
            air_resistance_coeff,
            l,
            None,
        );

        assert_eq!(
            cart_pole.parameters(),
            (
                pole_mass,
                cart_mass,
                friction_coeff,
                air_resistance_coeff,
                l
            )
        );
    }

    #[test]
    fn test_cart_pole_creation_with_registry() {
        let pole_mass = 1.0;
        let cart_mass = 2.0;
        let friction_coeff = 0.0;
        let air_resistance_coeff = 0.0;
        let l = 0.5;
        let registry = Arc::new(ExprRegistry::new());

        let cart_pole = CartPole::new(
            pole_mass,
            cart_mass,
            friction_coeff,
            air_resistance_coeff,
            l,
            Some(&registry),
        );

        assert_eq!(
            cart_pole.parameters(),
            (
                pole_mass,
                cart_mass,
                friction_coeff,
                air_resistance_coeff,
                l
            )
        );
        assert!(registry.get_scalar(c::MASS_POLE_SYMBOLIC).is_ok());
        assert!(registry.get_scalar(c::MASS_CART_SYMBOLIC).is_ok());
        assert!(
            registry
                .get_scalar(c::AIR_RESISTANCE_COEFF_SYMBOLIC)
                .is_ok()
        );
        assert!(registry.get_scalar(c::FRICTION_COEFF_SYMBOLIC).is_ok());
        assert!(registry.get_scalar(c::LENGTH_SYMBOLIC).is_ok());
        assert!(registry.get_scalar(c::GRAVITY_SYMBOLIC).is_ok());
        assert!(registry.get_vector(c::STATE_SYMBOLIC).is_ok());
    }

    #[test]
    fn test_cart_pole_parameters() {
        let cart_pole = CartPole::new(1.0, 2.0, 0.0, 0.0, 0.5, None);

        let (pole_mass, cart_mass, friction_coeff, air_resistance_coeff, l) =
            cart_pole.parameters();
        assert_eq!(pole_mass, 1.0);
        assert_eq!(cart_mass, 2.0);
        assert_eq!(friction_coeff, 0.0);
        assert_eq!(air_resistance_coeff, 0.0);
        assert_eq!(l, 0.5);
    }
}
