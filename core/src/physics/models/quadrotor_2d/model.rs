use super::state::Quadrotor2DState;
use crate::numeric_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::constants as c;
use crate::utils::Labelizable;
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
pub struct Quadrotor2D {
    m: f64,
    j: f64,
    l: f64,
}

impl Quadrotor2D {
    pub fn new(
        m: f64,
        j: f64,
        l: f64,
        registry: Option<&Arc<ExprRegistry>>,
        store_params: bool,
    ) -> Self {
        let model = Quadrotor2D { m, l, j };
        if let Some(registry) = registry {
            if store_params {
                model.store_params(registry);
            }
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            registry.insert_vector(c::STATE_SYMBOLIC, Quadrotor2DState::labels());
            registry.insert_vector_expr(c::INPUT_SYMBOLIC, ExprVector::new(&["u1", "u2"]));
            registry.insert_var("u1", 0.0);
            registry.insert_var("u2", 0.0);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadrotor_2d_new() {
        let quad = Quadrotor2D::new(1.0, 2.0, 1.5, None, true);
        let [m, j, l] = quad.extract(&["m", "j", "l"]);

        assert_eq!(m, 1.0);
        assert_eq!(j, 2.0);
        assert_eq!(l, 1.5);
    }

    #[test]
    fn test_quadrotor_2d_parameters() {
        let quad = Quadrotor2D::new(1.0, 2.0, 1.4, None, true);

        let params = quad.extract(&["m", "j", "l"]);
        assert_eq!(params, [1.0, 2.0, 1.4]);
    }
}
