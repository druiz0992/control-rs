use crate::SolverError;
use crate::dtos::LineSeachConfig;
use nalgebra::DVector;
use symbolic_services::symbolic::{SymbolicEvalResult, SymbolicFunction};

/// A struct that implements a line search algorithm for optimization problems.
///
/// The `LineSearch` struct is used to iteratively find a step size (`alpha`) that
/// minimizes a given merit function along a specified search direction. It uses
/// a backtracking approach, reducing the step size by a factor until a sufficient
/// decrease in the merit function is observed or the maximum number of iterations
/// is reached.
pub struct LineSearch {
    /// The maximum number of iterations allowed for the line search.
    max_iters: usize,
    ///  The factor by which the step size is reduced in each iteration.
    factor: f64,
}

impl LineSearch {
    /// A new `LineSearch` instance.
    pub fn new(opts: LineSeachConfig) -> Self {
        Self {
            max_iters: opts.get_max_iters(),
            factor: opts.get_factor(),
        }
    }

    /// Executes the line search algorithm to find an appropriate step size (`alpha`).
    pub fn run(
        &self,
        merit_fn: &SymbolicFunction,
        delta: &DVector<f64>,
        z: &[f64],
        default: Option<f64>,
    ) -> Result<f64, SolverError> {
        let mut alpha = 1.0;
        let current_merit = match default {
            Some(v) => SymbolicEvalResult::Scalar(v),
            _ => merit_fn.eval(z)?,
        };

        // new_z = z + delta * alpha
        let mut new_z = step_from(delta, z, alpha);

        for _ in 0..self.max_iters {
            let trial_merit = merit_fn.eval(new_z.as_slice())?;

            if trial_merit < current_merit {
                return Ok(alpha);
            } else {
                alpha *= self.factor;
                // new_z = z + delta * alpha
                new_z = step_from(delta, z, alpha);
            }
        }

        //Err(ModelError::SolverError(
        //    "Maximum number of iterations reached in linesearch".to_string(),
        //))
        Ok(alpha)
    }
}

/// Computes a new point by moving along the search direction scaled by the step size.
fn step_from(delta: &DVector<f64>, z: &[f64], alpha: f64) -> DVector<f64> {
    DVector::from_column_slice(z) + delta * alpha
}
