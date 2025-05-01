use crate::numeric_services::symbolic::{ExprRegistry, ExprVector, SymbolicFn};
use crate::physics::ModelError;
use crate::physics::traits::FromSymbolicEvalResult;
use nalgebra::DVector;
use std::collections::HashMap;
use std::sync::Arc;

use super::models::LineSeachConfig;

pub struct LineSearch;

impl LineSearch {
    pub fn new() -> Self {
        Self
    }

    pub fn run(
        &self,
        merit_fn: SymbolicFn,
        delta_z: DVector<f64>,
        registry: &Arc<ExprRegistry>,
        opts: LineSeachConfig,
    ) -> Result<f64, ModelError> {
        let mut alpha = opts.get_init_alpha();
        let max_iters = opts.get_max_iters();
        let factor = opts.get_factor();

        let x1 = registry.get_var("x1").unwrap();
        let x2 = registry.get_var("x2").unwrap();

        let mut t = &delta_z * alpha;
        let mut new_point = DVector::from_vec(vec![x1, x2]) + t;

        let current = (merit_fn)(None).map_err(|_| ModelError::EvaluationError)?;

        let mut tmp_registry = HashMap::<String, f64>::new();

        for _ in 0..max_iters {
            tmp_registry.insert("x1".to_string(), new_point[0]);
            tmp_registry.insert("x2".to_string(), new_point[1]);

            let candidate =
                (merit_fn)(Some(&tmp_registry)).map_err(|_| ModelError::EvaluationError)?;
            if candidate < current {
                return Ok(alpha);
            } else {
                alpha = alpha * factor;
                t = &delta_z * alpha;
                new_point = DVector::from_vec(vec![x1, x2]) + t;
            }
        }

        return Err(ModelError::SolverError(
            "Maximum number of iterations reached in linesearch".to_string(),
        ));
    }
}
