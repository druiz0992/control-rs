use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::GRAVITY as G;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone)]
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
        if let Some(registry) = registry {
            registry.insert_scalar("m1", ExprScalar::new(m1.to_string()));
            registry.insert_scalar("m2", ExprScalar::new(m2.to_string()));
            registry.insert_scalar("l1", ExprScalar::new(l1.to_string()));
            registry.insert_scalar("l2", ExprScalar::new(l2.to_string()));
            registry.insert_scalar(
                "air_resistance_coeff",
                ExprScalar::new(air_resistance_coeff.to_string()),
            );
            registry.insert_scalar("g", ExprScalar::new(G.to_string()));

            registry.insert_vector(
                "state",
                ExprVector::new(&["theta1", "omega1", "theta2", "omega2"]),
            );
            registry.insert_vector("input", ExprVector::new(&["u1", "u2"]));
            registry.insert_var("u1", 0.0);
            registry.insert_var("u2", 0.0);
        }

        DoublePendulum {
            m1,
            m2,
            l1,
            l2,
            air_resistance_coeff,
        }
    }

    pub fn parameters(&self) -> (f64, f64, f64, f64, f64) {
        (
            self.m1,
            self.m2,
            self.l1,
            self.l2,
            self.air_resistance_coeff,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_double_pendulum_new() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);
        let (m1, m2, l1, l2, air_resistance_coeff) = pendulum.parameters();

        assert_eq!(m1, 1.0);
        assert_eq!(m2, 2.0);
        assert_eq!(l1, 1.5);
        assert_eq!(l2, 2.5);
        assert_eq!(air_resistance_coeff, 0.0);
    }

    #[test]
    fn test_double_pendulum_parameters() {
        let pendulum = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, None);

        let params = pendulum.parameters();
        assert_eq!(params, (1.0, 2.0, 1.5, 2.5, 0.0));
    }
}
