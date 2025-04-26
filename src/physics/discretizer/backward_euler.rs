use crate::physics::traits::{Discretizer, Dynamics};
use crate::solver::Solver;

// Backward Euler residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - dt * f(x_{k+1})
//
// The root of R(x_{k+1}) gives the next state satisfying the Backward Euler integration scheme.

pub struct BackwardEuler<S> {
    solver: S,
}

impl<S> BackwardEuler<S> {
    pub fn new(solver: S) -> Self {
        BackwardEuler { solver }
    }
}

impl<D, S> Discretizer<D> for BackwardEuler<S>
where
    D: Dynamics,
    S: Solver<D::State>,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let f = |x_next: &D::State| x_next.clone() - state.clone() - model.dynamics(x_next) * dt;

        self.solver.solve(f, state.clone())
    }
}
