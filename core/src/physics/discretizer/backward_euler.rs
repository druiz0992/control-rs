use super::utils::{get_states, step_intrinsic};
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::traits::{Describable, Discretizer, Dynamics, State};
use crate::physics::{ModelError, constants as c};
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

// Backward Euler residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - dt * f(x_{k+1})
//
// The root of R(x_{k+1}) gives the next state satisfying the Backward Euler integration scheme.

pub struct BackwardEuler<D: Dynamics> {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolver,
    model: D,
}

impl<D: Dynamics> BackwardEuler<D> {
    pub fn new(model: D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let (current_state, next_state) = get_states(&registry)?;

        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let dyn_next_state = model
            .dynamics_symbolic(&next_state.wrap(), &registry)
            .scale(&dt_expr);

        let residual = next_state.sub(&current_state).sub(&dyn_next_state);
        let solver = NewtonSolver::new_root_solver(&residual, &next_state, &registry, None)?;

        Ok(BackwardEuler {
            registry,
            solver,
            model,
        })
    }
}

impl<D: Dynamics> Discretizer<D> for BackwardEuler<D> {
    fn step(
        &mut self,
        state: &D::State,
        _input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        step_intrinsic(state, dt, &self.solver, &self.registry)
    }

    fn get_model(&self) -> &D {
        &self.model
    }
}

impl<D: Dynamics> Describable for BackwardEuler<D> {
    fn name(&self) -> &'static str {
        "Backward-Euler"
    }
}
