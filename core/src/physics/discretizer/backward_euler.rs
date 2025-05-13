use super::utils;
use crate::common::Labelizable;
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
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
        let solver;
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let (current_state, next_state) = utils::get_states(&registry)?;

        if let Some(linear_term) = model.linear_term(&dt_expr, &registry) {
            let (_, next_v_state) = utils::get_v_states(&registry)?;
            let (current_q_state, _) = utils::get_q_states(&registry)?;
            let mass_matrix = registry.get_matrix(c::MASS_MATRIX_SYMBOLIC)?;
            let objective_expr = utils::build_objective(&mass_matrix, &linear_term, &next_v_state);
            let jacobian_constraints_expr = registry.get_vector(c::CONSTRAINT_JACOBIAN_SYMBOLIC)?;
            let ineq_constraints_expr = ExprVector::from_vec(vec![
                jacobian_constraints_expr
                    .dot(&current_q_state.add(&next_v_state.scale(&dt_expr)))
                    .unwrap()
                    .wrap(),
            ]);

            solver = NewtonSolver::new_minimization(
                &objective_expr,
                None,
                Some(ineq_constraints_expr),
                &next_state,
                &registry,
                None,
            )?;
        } else {
            let dyn_next_state = model
                .dynamics_symbolic(&next_state.wrap(), &registry)
                .scale(&dt_expr);

            let residual = next_state.sub(&current_state).sub(&dyn_next_state);
            solver = NewtonSolver::new_root_solver(&residual, &next_state, &registry, None)?;
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
        let (mut next_state, _multipliers) =
            utils::step_intrinsic(state, dt, &self.solver, &self.registry)?;
        let v_dims = D::State::dim_v();
        if v_dims > 0 {
            let labels = D::State::labels();
            let q_dims = D::State::dim_q();
            let next_v = next_state.vectorize(&labels[q_dims..]);
            let mut current_q = state.vectorize(&labels[..q_dims]);
            current_q = current_q
                .iter()
                .zip(next_v.iter())
                .map(|(q, v)| q + dt * v)
                .collect();
            current_q.extend_from_slice(&next_v);
            next_state = D::State::from_vec(current_q.clone());
        }
        Ok(next_state)
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
