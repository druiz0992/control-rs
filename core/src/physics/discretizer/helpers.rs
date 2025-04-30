use crate::numeric_services::symbolic::{ExprRegistry, SymbolicFn};
use crate::physics::ModelError;
use crate::physics::models::state::State;
use crate::physics::traits::Dynamics;
use crate::solver::Solver;
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

pub fn symbolic_intrinsic_step<D: Dynamics>(
    registry: &Arc<ExprRegistry>,
    residual_func: &SymbolicFn,
    jacobian_func: &SymbolicFn,
    state: &D::State,
    dt: f64,
) -> Result<D::State, ModelError> {
    registry.insert_var("dt", dt);
    let state_vec = state.as_vec();
    let state_components = registry
        .get_vector("state")
        .map_err(|e| ModelError::Symbolic(e.to_string()))?
        .as_vec();
    for (name, value) in state_components.iter().zip(state_vec.iter()) {
        registry.insert_var(name.as_str(), *value);
    }

    let next_state_components = registry
        .get_vector("next_state")
        .map_err(|e| ModelError::Symbolic(e.to_string()))?
        .as_vec();
    for (name, value) in next_state_components.iter().zip(state_vec.iter()) {
        registry.insert_var(name.as_str(), *value);
    }

    let solver = NewtonSolver::new();
    Ok(solver.solve(residual_func, jacobian_func, state, Arc::clone(registry)))
}
