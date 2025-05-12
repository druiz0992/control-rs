use super::state::SlidingBrickState;
use crate::common::Labelizable;
use crate::numeric_services::symbolic::ExprRegistry;
use crate::physics::constants as c;
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
pub struct SlidingBrick {
    m: f64,
    friction_coeff: f64,
}

impl SlidingBrick {
    pub fn new(m: f64, friction_coeff: f64, registry: Option<&Arc<ExprRegistry>>) -> Self {
        if let Some(registry) = registry {
            registry.insert_scalar(c::MASS_SYMBOLIC, m);
            registry.insert_scalar(c::FRICTION_COEFF_SYMBOLIC, friction_coeff);
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            registry.insert_vector(c::STATE_SYMBOLIC, SlidingBrickState::labels());
        }

        SlidingBrick { m, friction_coeff }
    }
}
