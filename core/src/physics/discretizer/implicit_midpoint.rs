use crate::physics::traits::{Discretizer, Dynamics};
use crate::solver::Solver;

// residual(x_next) = x_next - x_k - dt * f((x_k + x_next) / 2)

pub struct ImplicitModpoint<S> {
    solver: S,
}

impl<S> ImplicitModpoint<S> {
    pub fn new(solver: S) -> Self {
        ImplicitModpoint { solver }
    }
}

impl<D, S> Discretizer<D> for ImplicitModpoint<S>
where
    D: Dynamics,
    S: Solver<D::State>,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let f = |x_next: &D::State| {
            let x_mid = (state.clone() + x_next.clone()) * 0.5;
            let f_mid = model.dynamics(&x_mid);

            x_next.clone() - state.clone() - f_mid * dt
        };

        //self.solver.solve(f, state.clone())
        todo!();
    }
}
