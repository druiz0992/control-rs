use super::state::BouncingBallState;
use crate::utils::Labelizable;
use crate::numeric_services::symbolic::{ExprMatrix, ExprRegistry, ExprScalar, ExprVector};
use crate::physics::constants as c;
use crate::physics::traits::State;
use macros::LabelOps;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Clone, LabelOps)]
pub struct BouncingBall {
    m: f64,
    friction_coeff: f64,
}

impl BouncingBall {
    pub fn new(m: f64, friction_coeff: f64, registry: Option<&Arc<ExprRegistry>>) -> Self {
        if let Some(registry) = registry {
            registry.insert_scalar(c::MASS_SYMBOLIC, m);
            registry.insert_scalar(c::FRICTION_COEFF_SYMBOLIC, friction_coeff);
            registry.insert_scalar(c::GRAVITY_SYMBOLIC, c::GRAVITY);
            let labels = BouncingBallState::labels();
            registry.insert_vector(c::STATE_SYMBOLIC, labels);

            let dim_q = BouncingBallState::dim_q();
            registry.insert_vector(c::STATE_Q_SYMBOLIC, &labels[..dim_q]);
            registry.insert_vector(c::STATE_V_SYMBOLIC, &labels[dim_q..]);

            // constaints
            let m = registry.get_scalar(c::MASS_SYMBOLIC).unwrap();
            registry.insert_matrix_expr(c::MASS_MATRIX_SYMBOLIC, ExprMatrix::identity(2).scale(&m));

            let one = ExprScalar::one();
            let zero = ExprScalar::zero();
            registry.insert_vector_expr(
                c::CONSTRAINT_JACOBIAN_SYMBOLIC,
                ExprVector::from_vec(vec![zero, one]),
            );
        }

        BouncingBall { m, friction_coeff }
    }
}
