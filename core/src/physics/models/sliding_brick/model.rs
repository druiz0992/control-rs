use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use crate::physics::GRAVITY as G;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SlidingBrick {
    m: f64,
    friction_coeff: f64,
}

impl SlidingBrick {
    pub fn new(m: f64, friction_coeff: f64, registry: Option<&Arc<ExprRegistry>>) -> Self {
        if let Some(registry) = registry {
            registry.insert_scalar("m", ExprScalar::new(m.to_string()));
            registry.insert_scalar(
                "friction_coeff",
                ExprScalar::new(friction_coeff.to_string()),
            );
            registry.insert_scalar("g", ExprScalar::new(G.to_string()));

            registry.insert_vector("state", ExprVector::new(&["pos_x", "pos_y", "v_x", "v_y"]));
        }

        SlidingBrick { m, friction_coeff }
    }

    pub fn parameters(&self) -> (f64, f64) {
        (self.m, self.friction_coeff)
    }
}
