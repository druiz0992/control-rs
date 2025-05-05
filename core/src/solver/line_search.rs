use crate::numeric_services::symbolic::{SymbolicEvalResult, SymbolicFunction};
use crate::physics::ModelError;
use nalgebra::DVector;

use super::models::LineSeachConfig;

pub struct LineSearch {
    max_iters: usize,
    factor: f64,
}

impl LineSearch {
    pub fn new(opts: LineSeachConfig) -> Self {
        Self {
            max_iters: opts.get_max_iters(),
            factor: opts.get_factor(),
        }
    }

    pub fn run(
        &self,
        merit_fn: &SymbolicFunction,
        search_direction: &DVector<f64>,
        start_point: &[f64],
    ) -> Result<f64, ModelError> {
        let mut alpha = 1.0;
        let current_merit = merit_fn
            .eval(start_point)
            .map_err(|_| ModelError::EvaluationError)?;

        let mut new_point = compute_new_point(search_direction, start_point, alpha);

        for _ in 0..self.max_iters {
            let trial_merit = merit_fn
                .eval(new_point.as_slice())
                .map_err(|_| ModelError::EvaluationError)?;

            if trial_merit < current_merit {
                return Ok(alpha);
            } else {
                //return Ok(alpha);
                alpha = alpha * self.factor;
                new_point = compute_new_point(search_direction, start_point, alpha);
            }
        }

        //return Ok(alpha);
        return Err(ModelError::SolverError(
            "Maximum number of iterations reached in linesearch".to_string(),
        ));
    }
}

fn compute_new_point(
    search_direction: &DVector<f64>,
    start_point: &[f64],
    alpha: f64,
) -> DVector<f64> {
    let scaled_direction = search_direction * alpha;
    DVector::from_column_slice(start_point) + scaled_direction
}
