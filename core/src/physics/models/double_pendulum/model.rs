use crate::physics::constants as c;
use crate::physics::models::{DoublePendulumInput, DoublePendulumState};
use crate::utils::{Identifiable, Labelizable};
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use symbolic_services::symbolic::ExprRegistry;

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
pub struct DoublePendulum {
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,
    air_resistance_coeff: f64,
}

impl DoublePendulum {
    pub fn new(
        m1: f64,
        m2: f64,
        l1: f64,
        l2: f64,
        air_resistance_coeff: f64,
        registry: Option<&Arc<ExprRegistry>>,
    ) -> Self {
        let model = DoublePendulum {
            m1,
            m2,
            l1,
            l2,
            air_resistance_coeff,
        };
        if let Some(registry) = registry {
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            registry.insert_vector(c::STATE_SYMBOLIC, DoublePendulumState::labels());
            registry.insert_vector(c::INPUT_SYMBOLIC, DoublePendulumInput::labels());
            registry.insert_vector(c::MODEL_SYMBOLIC, DoublePendulum::labels());
        }
        model
    }
}

impl Identifiable for DoublePendulum {
    fn name() -> &'static str {
        "double_pendulum"
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_double_pendulum_new() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);
        let [m1, m2, l1, l2, air_resistance_coeff] =
            pendulum.extract(&["m1", "m2", "l1", "l2", "air_resistance_coeff"]);

        assert_eq!(m1, 1.0);
        assert_eq!(m2, 2.0);
        assert_eq!(l1, 1.5);
        assert_eq!(l2, 2.5);
        assert_eq!(air_resistance_coeff, 0.0);
    }

    #[test]
    fn test_double_pendulum_parameters() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);

        let params = pendulum.extract(&["m1", "m2", "l1", "l2", "air_resistance_coeff"]);
        assert_eq!(params, [1.0, 2.0, 1.5, 2.5, 0.0]);
    }
}
