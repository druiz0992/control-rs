use crate::numeric_services::symbolic::{
    ExprRegistry, ExprScalar, ExprVector, SymbolicExpr, SymbolicFn,
};
use crate::physics::traits::{Discretizer, Dynamics, State};
use crate::solver::Solver;
use crate::solver::newton::NewtonSolver;
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
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Self {
        let dt_expr = ExprScalar::new("dt");
        let current_state = registry.get_vector("state").unwrap();
        let next_state = ExprVector::new(
            &current_state
                .as_vec()
                .iter()
                .map(|e| format!("next_{}", e.as_str()))
                .collect::<Vec<String>>()
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>(),
        );
        registry.insert_vector("next_state", next_state.clone());

        let dyn_next_state = model
            .dynamics_symbolic(next_state.clone().wrap(), &registry)
            .scale(&dt_expr);

        let residual = next_state.sub(&current_state).sub(&dyn_next_state);
        let jacobian = residual.jacobian(&next_state).unwrap();

        let residual_func = residual.to_fn(&registry).unwrap();
        let jacobian_func = jacobian.to_fn(&registry).unwrap();
        BackwardEuler {
            registry,
            residual_func,
            jacobian_func,
        }
    }
}

impl<D> Discretizer<D> for BackwardEuler
where
    D: Dynamics,
{
    fn step(&mut self, _model: &D, state: &D::State, dt: f64) -> D::State {
        self.registry.insert_var("dt".into(), dt);
        let state_vec = state.as_vec();
        let state_components = self.registry.get_vector("state").unwrap().as_vec();
        for (name, value) in state_components.iter().zip(state_vec.iter()) {
            self.registry.insert_var(name.as_str(), *value);
        }
        let next_state_components = self.registry.get_vector("next_state").unwrap().as_vec();
        for (name, value) in next_state_components.iter().zip(state_vec.iter()) {
            self.registry.insert_var(name.as_str(), *value);
        }

        let solver = NewtonSolver::new(100, 1e-6);

        solver.solve(
            &self.residual_func,
            &self.jacobian_func,
            state,
            Arc::clone(&self.registry),
        )
    }
}
