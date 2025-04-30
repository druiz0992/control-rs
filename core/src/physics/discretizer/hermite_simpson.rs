use crate::physics::traits::{Discretizer, Dynamics};
use crate::solver::Solver;

// Hermite–Simpson residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - (dt / 6) * (f_k + 4 * f_m + f_{k+1})
//
// where:
//   f_k     = f(x_k)
//   f_{k+1} = f(x_{k+1})
//   x_m     = 0.5 * (x_k + x_{k+1}) + (dt / 8) * (f_k - f_{k+1})
//   f_m     = f(x_m)
//
// The root of R(x_{k+1}) gives the next state satisfying the Hermite–Simpson integration scheme.

pub struct HermiteSimpson<S> {
    solver: S,
}

impl<S> HermiteSimpson<S> {
    pub fn new(solver: S) -> Self {
        HermiteSimpson { solver }
    }
}

impl<D, S> Discretizer<D> for HermiteSimpson<S>
where
    D: Dynamics,
    S: Solver<D::State>,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let f = |x_next: &D::State| {
            let f_k = model.dynamics(state);
            let f_next = model.dynamics(x_next);

            let x_mid = (state.clone() + x_next.clone()) * 0.5
                + (f_k.clone() - f_next.clone()) * (dt / 8.0);

            let f_mid = model.dynamics(&x_mid);

            let delta = (f_k + f_mid * 4.0 + f_next) * (dt / 6.0);

            x_next.clone() - state.clone() - delta
        };

        //self.solver.solve(f, state.clone())
        todo!();
    }
}
