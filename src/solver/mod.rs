pub mod newton;

pub trait Solver<S> {
    fn solve<F>(&mut self, f: F, initial_guess: S) -> S
    where
        F: Fn(&S) -> S;
}
