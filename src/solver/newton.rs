use super::Solver;
use crate::physics::traits::State;

pub struct NewtonSolver {
    pub max_iters: usize,
    pub tolerance: f64,
}

impl<S> Solver<S> for NewtonSolver
where
    S: State,
{
    fn solve<F>(&mut self, f: F, mut x: S) -> S
    where
        F: Fn(&S) -> S,
    {
        for _ in 0..self.max_iters {
            let fx = f(&x);
            if fx.as_vec().iter().map(|v| v.abs()).sum::<f64>() < self.tolerance {
                break;
            }

            // TODO: Numerical Jacobian + Newton step.
            // For now, just a placeholder update.
            x = x - fx * 0.01;
        }
        x
    }
}
