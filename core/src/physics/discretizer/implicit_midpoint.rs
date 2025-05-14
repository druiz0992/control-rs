use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::models::state::State;
use crate::physics::traits::{Describable, Discretizer, Dynamics};
use crate::physics::{ModelError, constants as c};
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

use super::utils::{get_states, step_intrinsic};

// residual(x_next) = x_next - x_k - dt * f((x_k + x_next) / 2)

pub struct ImplicitMidpoint<D: Dynamics> {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolver,
    model: D,
}

impl<D: Dynamics> ImplicitMidpoint<D> {
    pub fn new(model: D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let (current_state, next_state) = get_states(&registry)?;

        let mid_state = current_state.add(&next_state).wrap().scalef(0.5).wrap();

        let dyn_mid_state = model
            .dynamics_symbolic(&mid_state, &registry)
            .wrap()
            .scale(&dt_expr);

        let residual = next_state.sub(&current_state).sub(&dyn_mid_state).wrap();
        let solver = NewtonSolver::new_root_solver(&residual, &next_state, &registry, None)?;

        Ok(ImplicitMidpoint {
            registry,
            solver,
            model,
        })
    }
}

impl<D: Dynamics> Discretizer<D> for ImplicitMidpoint<D> {
    fn step(
        &mut self,
        state: &D::State,
        _input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let (new_state, _multipliers) = step_intrinsic(state, dt, &self.solver, &self.registry)?;
        Ok(D::State::from_vec(new_state))
    }

    fn get_model(&self) -> &D {
        &self.model
    }
}

impl<D: Dynamics> Describable for ImplicitMidpoint<D> {
    fn name(&self) -> &'static str {
        "Implicit-Midpoint"
    }
}
