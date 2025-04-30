use super::helpers::symbolic_intrinsic_step;
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, SymbolicExpr, SymbolicFn};
use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics};
use std::sync::Arc;

// Backward Euler residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - dt * f(x_{k+1})
//
// The root of R(x_{k+1}) gives the next state satisfying the Backward Euler integration scheme.

pub struct BackwardEuler {
    registry: Arc<ExprRegistry>,
    residual_func: SymbolicFn,
    jacobian_func: SymbolicFn,
}

impl BackwardEuler {
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let dt_expr = ExprScalar::new("dt");
        let current_state = registry
            .get_vector("state")
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let next_state = current_state.next_state();
        registry.insert_vector("next_state", next_state.clone());

        let dyn_next_state = model
            .dynamics_symbolic(next_state.clone().wrap(), &registry)
            .scale(&dt_expr);

        let residual = next_state.sub(&current_state).sub(&dyn_next_state);
        let jacobian = residual
            .jacobian(&next_state)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        let residual_func = residual
            .to_fn(&registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;
        let jacobian_func = jacobian
            .to_fn(&registry)
            .map_err(|e| ModelError::Symbolic(e.to_string()))?;

        Ok(BackwardEuler {
            registry,
            residual_func,
            jacobian_func,
        })
    }
}

impl<D> Discretizer<D> for BackwardEuler
where
    D: Dynamics,
{
    fn step(&mut self, _model: &D, state: &D::State, dt: f64) -> Result<D::State, ModelError> {
        symbolic_intrinsic_step::<D>(
            &self.registry,
            &self.residual_func,
            &self.jacobian_func,
            state,
            dt,
        )
    }
}

impl Describable for BackwardEuler {
    fn name(&self) -> &'static str {
        "Backward-Euler"
    }
}
