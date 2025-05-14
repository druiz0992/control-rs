use super::utils;
use crate::common::Labelizable;
use crate::numeric_services::solver::{NewtonSolver, OptimizerConfig};
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::traits::{Describable, Discretizer, Dynamics, State};
use crate::physics::{ModelError, constants as c};
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
    pub fn new(
        model: D,
        registry: Arc<ExprRegistry>,
        solver_options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let solver;
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let (current_state, next_state) = utils::get_states(&registry)?;

        if let Some(linear_term) = model.linear_term(&dt_expr, &registry) {
            solver = utils::init_constrained_dynamics(
                &linear_term,
                &dt_expr,
                solver_options,
                &registry,
            )?;
        } else {
            let dyn_next_state = model
                .dynamics_symbolic(&next_state.wrap(), &registry)
                .scale(&dt_expr);

            let residual = next_state.sub(&current_state).sub(&dyn_next_state);
            solver =
                NewtonSolver::new_root_solver(&residual, &next_state, &registry, solver_options)?;
        }

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
        let v_dims = D::State::dim_v();
        let (next_v, _) = utils::step_intrinsic(state, dt, &self.solver, &self.registry)?;
        if v_dims == 0 {
            return Ok(D::State::from_vec(next_v));
        }

        let labels = D::State::labels();
        let q_slice = &labels[..D::State::dim_q()];

        let current_q = state.vectorize(q_slice);
        let next_q: Vec<_> = current_q
            .iter()
            .zip(next_v.iter())
            .map(|(q, v)| q + dt * v)
            .collect();
        let mut full_state = next_q;
        full_state.extend_from_slice(&next_v);

        Ok(D::State::from_vec(full_state))
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
