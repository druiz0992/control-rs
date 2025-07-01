use super::state::CartPoleState;
use symbolic_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::constants as c;
use crate::utils::{Identifiable, Labelizable};
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
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
        l: f64,
        friction_coeff: f64,
        air_resistance_coeff: f64,
        registry: Option<&Arc<ExprRegistry>>,
        store_params: bool,
    ) -> Self {
        let model = CartPole {
            pole_mass,
            cart_mass,
            friction_coeff,
            air_resistance_coeff,
            l,
        };
        if let Some(registry) = registry {
            if store_params {
                model.store_params(registry);
            }
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            registry.insert_vector(c::STATE_SYMBOLIC, CartPoleState::labels());
            registry.insert_vector_expr(c::INPUT_SYMBOLIC, ExprVector::new(&["u1"]));
            registry.insert_var("u1", 0.0);
        }
        model
    }
    pub fn store_params(&self, registry: &Arc<ExprRegistry>) {
        let labels = Self::labels();
        let params = self.vectorize(labels);

        labels
            .iter()
            .zip(params.iter())
            .for_each(|(n, v)| registry.insert_scalar(n, *v));
    }
}

impl Identifiable for CartPole {
    fn name() -> &'static str {
        "cartpole"
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
            l,
            friction_coeff,
            air_resistance_coeff,
            None,
            true,
        );

        let params = cart_pole.extract(&[
            "pole_mass",
            "cart_mass",
            "friction_coeff",
            "air_resistance_coeff",
            "l",
        ]);

        assert_eq!(
            params,
            [
                pole_mass,
                cart_mass,
                friction_coeff,
                air_resistance_coeff,
                l
            ]
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
            l,
            friction_coeff,
            air_resistance_coeff,
            Some(&registry),
            true,
        );

        let params = cart_pole.extract(&[
            "pole_mass",
            "cart_mass",
            "friction_coeff",
            "air_resistance_coeff",
            "l",
        ]);
        assert_eq!(
            params,
            [
                pole_mass,
                cart_mass,
                friction_coeff,
                air_resistance_coeff,
                l
            ]
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
        let cart_pole = CartPole::new(1.0, 2.0, 0.5, 0.0, 0.0, None, true);

        let [
            pole_mass,
            cart_mass,
            friction_coeff,
            air_resistance_coeff,
            l,
        ] = cart_pole.extract(&[
            "pole_mass",
            "cart_mass",
            "friction_coeff",
            "air_resistance_coeff",
            "l",
        ]);
        assert_eq!(pole_mass, 1.0);
        assert_eq!(cart_mass, 2.0);
        assert_eq!(friction_coeff, 0.0);
        assert_eq!(air_resistance_coeff, 0.0);
        assert_eq!(l, 0.5);
    }
}
