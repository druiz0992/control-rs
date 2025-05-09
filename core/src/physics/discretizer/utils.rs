use crate::numeric_services::symbolic::{ExprRegistry, ExprVector};
use crate::physics::constants as c;
use crate::physics::error::ModelError;
use crate::physics::traits::State;
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

pub fn get_states(registry: &Arc<ExprRegistry>) -> Result<(ExprVector, ExprVector), ModelError> {
    let current_state = registry.get_vector(c::STATE_SYMBOLIC)?;
    let next_state = current_state.build_next();
    registry.insert_vector_expr(c::NEXT_STATE_SYMBOLIC, next_state.clone());

    Ok((current_state, next_state))
}

pub fn step_intrinsic<S: State>(
    state: &S,
    dt: f64,
    solver: &NewtonSolver,
    registry: &Arc<ExprRegistry>,
) -> Result<S, ModelError> {
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    registry.insert_vec_as_vars(c::STATE_SYMBOLIC, &state.as_vec())?;
    registry.insert_vec_as_vars(c::NEXT_STATE_SYMBOLIC, &state.as_vec())?;

    let history = solver.solve(&state.as_vec(), registry)?;
    Ok(State::from_vec(history.last().cloned().ok_or_else(
        || ModelError::SolverError(String::from("Solver failed")),
    )?))
}
