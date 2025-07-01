use super::state::Quadrotor2DState;
use crate::physics::constants as c;
use crate::physics::models::Quadrotor2DInput;
use crate::utils::{Identifiable, Labelizable};
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use symbolic_services::symbolic::{ExprRegistry};

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
pub struct Quadrotor2D {
    m: f64,
    j: f64,
    l: f64,
}

impl Quadrotor2D {
    pub fn new(m: f64, j: f64, l: f64, registry: Option<&Arc<ExprRegistry>>) -> Self {
        let model = Self::base_new(m, j, l);
        if let Some(registry) = registry {
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            registry.insert_vector(c::STATE_SYMBOLIC, Quadrotor2DState::labels());
            registry.insert_vector(c::MODEL_SYMBOLIC, Quadrotor2D::labels());
            registry.insert_vector(c::INPUT_SYMBOLIC, Quadrotor2DInput::labels());
        }
        model
    }

    fn base_new(m: f64, j: f64, l: f64) -> Self {
        Quadrotor2D { m, l, j }
    }
}

impl Identifiable for Quadrotor2D {
    fn name() -> &'static str {
        "quadrotor_2d"
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadrotor_2d_new() {
        let quad = Quadrotor2D::new(1.0, 2.0, 1.5, None);
        let [m, j, l] = quad.extract(&["m", "j", "l"]);

        assert_eq!(m, 1.0);
        assert_eq!(j, 2.0);
        assert_eq!(l, 1.5);
    }

    #[test]
    fn test_quadrotor_2d_parameters() {
        let quad = Quadrotor2D::new(1.0, 2.0, 1.4, None);

        let params = quad.extract(&["m", "j", "l"]);
        assert_eq!(params, [1.0, 2.0, 1.4]);
    }
}
