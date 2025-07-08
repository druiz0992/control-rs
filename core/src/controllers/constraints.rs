use super::{ControllerInput, ControllerState};
use crate::physics::traits::State;
use crate::physics::{ModelError, traits::PhysicsSim};
use general::{matrix, vector};
use nalgebra::{DMatrix, DVector};

pub type ConstraintBound = (f64, f64);
pub type LowerBoundVector = DVector<f64>;
pub type UpperBoundVector = DVector<f64>;
pub type QpConstraints = (DVector<f64>, DMatrix<f64>, DVector<f64>);

/// Represents a transformation T for constraints, including lower and upper bounds
/// and a transformation matrix so that lb <= T * state <= ub.
///
/// This struct provides methods to create and manipulate constraint transformations
/// for both input and state dimensions. It supports uniform bounds, element-wise bounds,
/// and expanded bounds for multiple steps.

#[derive(Clone, Debug)]
pub struct ConstraintAffine {
    lb: DVector<f64>,
    ub: DVector<f64>,
    transform: DMatrix<f64>,
}

impl ConstraintAffine {
    /// Creates a new `ConstraintAffine` for input dimensions with specified bounds and transformation matrix.
    pub fn new_bounds_input<S: PhysicsSim>(
        lb: LowerBoundVector,
        ub: UpperBoundVector,
        transform: DMatrix<f64>,
    ) -> Result<Self, ModelError> {
        let input_dim = ControllerInput::<S>::dim_q();

        if lb.len() != input_dim {
            return Err(ModelError::ConfigError(
                "Bound vector lengths do not match with input vector dimension".into(),
            ));
        }
        Self::new_bounds(lb, ub, transform)
    }
    /// Creates a new `ConstraintAffine` for input dimensions with uniform bounds.
    pub fn new_uniform_bounds_input<S: PhysicsSim>(limit: ConstraintBound) -> Self {
        let input_dim = ControllerInput::<S>::dim_q();
        Self::new_uniform_bounds(limit, input_dim)
    }
    /// Creates a new `ConstraintAffine` for state dimensions with uniform bounds.
    pub fn new_uniform_bounds_state<S: PhysicsSim>(limit: ConstraintBound) -> Self {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        Self::new_uniform_bounds(limit, state_dim)
    }

    pub fn new_single_bound_input<S: PhysicsSim>(
        limit: ConstraintBound,
        idx: usize,
    ) -> Result<Self, ModelError> {
        let input_dim = ControllerInput::<S>::dim_q();
        if idx >= input_dim {
            return Err(ModelError::ConfigError(
                "Mistmatch in input vector length".into(),
            ));
        }
        let mut transform = DMatrix::zeros(1, input_dim);
        transform[(0, idx)] = 1.0;
        let lb = DVector::from_column_slice(&[limit.0]);
        let ub = DVector::from_column_slice(&[limit.1]);
        Self::new_bounds(lb, ub, transform)
    }

    pub fn new_single_bound_state<S: PhysicsSim>(
        limit: ConstraintBound,
        idx: usize,
    ) -> Result<Self, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        if idx >= state_dim {
            return Err(ModelError::ConfigError(
                "Mistmatch in state vector length".into(),
            ));
        }
        let mut transform = DMatrix::zeros(1, state_dim);
        transform[(0, idx)] = 1.0;
        let lb = DVector::from_column_slice(&[limit.0]);
        let ub = DVector::from_column_slice(&[limit.1]);
        Self::new_bounds(lb, ub, transform)
    }

    /// Creates a new `ConstraintAffine` for input dimensions with element-wise bounds.
    pub fn new_elementwise_bounds_input<S: PhysicsSim>(
        lb: LowerBoundVector,
        ub: UpperBoundVector,
    ) -> Result<Self, ModelError> {
        let input_dim = ControllerInput::<S>::dim_q();

        if lb.len() != input_dim {
            return Err(ModelError::ConfigError(
                "Bound vector lengths do not match with input vector dimension".into(),
            ));
        }
        Self::new_elementwise_bounds(lb, ub)
    }

    /// Creates a new `ConstraintAffine` for state dimensions with element-wise bounds.
    pub fn new_elementwise_bounds_state<S: PhysicsSim>(
        lb: LowerBoundVector,
        ub: UpperBoundVector,
    ) -> Result<Self, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();

        if lb.len() != state_dim {
            return Err(ModelError::ConfigError(
                "Bound vector lengths do not match with state vector dimension".into(),
            ));
        }
        Self::new_elementwise_bounds(lb, ub)
    }

    /// Expands the input constraint transformation and bounds to be compatible with qp_lqr controller's
    /// c (constraint matrix) and bounds
    pub fn expand_input<S: PhysicsSim>(&self, n_steps: usize) -> Result<QpConstraints, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();

        let expanded_transform = matrix::hstack(
            self.transform.clone(),
            DMatrix::zeros(self.transform.nrows(), state_dim),
        )
        .map_err(ModelError::ConfigError)?;
        let constraint_mat =
            matrix::kron(&DMatrix::identity(n_steps, n_steps), &expanded_transform);
        let (lb, ub) = self.expand_bounds(n_steps);

        Ok((lb, constraint_mat, ub))
    }
    /// Expands the state constraint transformation and bounds to be compatible with qp_lqr controller's
    /// c (constraint matrix) and bounds
    pub fn expand_state<S: PhysicsSim>(&self, n_steps: usize) -> Result<QpConstraints, ModelError> {
        let input_dim = ControllerInput::<S>::dim_q();

        let expanded_transform = matrix::hstack(
            DMatrix::zeros(self.transform.nrows(), input_dim),
            self.transform.clone(),
        )
        .map_err(ModelError::ConfigError)?;
        let constraint_mat =
            matrix::kron(&DMatrix::identity(n_steps, n_steps), &expanded_transform);
        let (lb, ub) = self.expand_bounds(n_steps);

        Ok((lb, constraint_mat, ub))
    }

    pub fn bounds_as_slice(&self) -> (&[f64], &[f64]) {
        (self.lb.as_slice(), self.ub.as_slice())
    }

    pub fn expand_bounds(&self, n_steps: usize) -> (DVector<f64>, DVector<f64>) {
        let one_v = DVector::from_column_slice(&vec![1.0; n_steps]);
        let lb = vector::kron(&one_v, &self.lb);
        let ub = vector::kron(&one_v, &self.ub);
        (lb, ub)
    }
    fn new_uniform_bounds(limit: ConstraintBound, dims: usize) -> Self {
        Self {
            lb: DVector::from_column_slice(&vec![limit.0; dims]),
            ub: DVector::from_column_slice(&vec![limit.1; dims]),
            transform: DMatrix::<f64>::identity(dims, dims),
        }
    }
    fn new_elementwise_bounds(
        lb: LowerBoundVector,
        ub: UpperBoundVector,
    ) -> Result<Self, ModelError> {
        let dim = lb.len();
        if lb.len() != ub.len() {
            return Err(ModelError::ConfigError(
                "Lower and Upper bound vector lengths mismatch.".into(),
            ));
        }
        Ok(Self {
            lb,
            ub,
            transform: DMatrix::<f64>::identity(dim, dim),
        })
    }
    fn new_bounds(
        lb: LowerBoundVector,
        ub: UpperBoundVector,
        transform: DMatrix<f64>,
    ) -> Result<Self, ModelError> {
        if lb.len() != ub.len() {
            return Err(ModelError::ConfigError(
                "Lower and Upper bound vector lengths mismatch.".into(),
            ));
        }
        Ok(Self { lb, ub, transform })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{discretizer::RK4Symbolic, models::Quadrotor2D, simulator::BasicSim};
    type MockPhysicsSim = BasicSim<Quadrotor2D, RK4Symbolic<Quadrotor2D>>;

    #[test]
    fn test_new_uniform_bounds_input() {
        let limit = (0.0, 1.0);

        let transform = ConstraintAffine::new_uniform_bounds_input::<MockPhysicsSim>(limit);

        assert_eq!(
            transform.lb.len(),
            ControllerInput::<MockPhysicsSim>::dim_q()
        );
        assert_eq!(
            transform.ub.len(),
            ControllerInput::<MockPhysicsSim>::dim_q()
        );
        assert_eq!(transform.lb[0], limit.0);
        assert_eq!(transform.ub[0], limit.1);
    }

    #[test]
    fn test_new_uniform_bounds_state() {
        let limit = (-1.0, 1.0);
        let transform = ConstraintAffine::new_uniform_bounds_state::<MockPhysicsSim>(limit);

        let expected_dim =
            ControllerState::<MockPhysicsSim>::dim_q() + ControllerState::<MockPhysicsSim>::dim_v();
        assert_eq!(transform.lb.len(), expected_dim);
        assert_eq!(transform.ub.len(), expected_dim);
        assert_eq!(transform.lb[0], limit.0);
        assert_eq!(transform.ub[0], limit.1);
    }

    #[test]
    fn test_new_elementwise_bounds_input() {
        let lb = DVector::from_column_slice(&[0.0, 0.1]);
        let ub = DVector::from_column_slice(&[1.0, 1.1]);

        let transform = ConstraintAffine::new_elementwise_bounds_input::<MockPhysicsSim>(
            lb.clone(),
            ub.clone(),
        )
        .unwrap();

        assert_eq!(transform.lb, lb);
        assert_eq!(transform.ub, ub);
    }

    #[test]
    fn test_new_elementwise_bounds_state() {
        let lb = DVector::from_column_slice(&[-1.0, -0.5, 0.0, 0.5, 0.3, -3.2]);
        let ub = DVector::from_column_slice(&[1.0, 1.5, 2.0, 2.5, 2.1, 3.1]);

        let transform = ConstraintAffine::new_elementwise_bounds_state::<MockPhysicsSim>(
            lb.clone(),
            ub.clone(),
        )
        .unwrap();

        assert_eq!(transform.lb, lb);
        assert_eq!(transform.ub, ub);
    }

    #[test]
    fn test_expand_bounds() {
        let lb = DVector::from_column_slice(&[0.0, 0.1]);
        let ub = DVector::from_column_slice(&[1.0, 1.1]);
        let transform = ConstraintAffine {
            lb: lb.clone(),
            ub: ub.clone(),
            transform: DMatrix::identity(2, 2),
        };

        let n_steps = 3;
        let (expanded_lb, expanded_ub) = transform.expand_bounds(n_steps);

        assert_eq!(expanded_lb.len(), lb.len() * n_steps);
        assert_eq!(expanded_ub.len(), ub.len() * n_steps);
    }

    #[test]
    fn test_expand_input() {
        let lb = DVector::from_column_slice(&[0.0, 0.1]);
        let ub = DVector::from_column_slice(&[1.0, 1.1]);
        let transform = ConstraintAffine {
            lb: lb.clone(),
            ub: ub.clone(),
            transform: DMatrix::identity(2, 2),
        };

        let n_steps = 3;
        let (expanded_lb, constraint_mat, expanded_ub) =
            transform.expand_input::<MockPhysicsSim>(n_steps).unwrap();

        assert_eq!(expanded_lb.len(), lb.len() * n_steps);
        assert_eq!(expanded_ub.len(), ub.len() * n_steps);
        assert_eq!(constraint_mat.nrows(), lb.len() * n_steps);
    }

    #[test]
    fn test_expand_state() {
        let lb = DVector::from_column_slice(&[0.0, 0.1, 0.2, 0.1, 1.0, -2.1]);
        let ub = DVector::from_column_slice(&[1.0, 1.1, 1.2, 0.0, 1.0, 0.1]);
        let transform = ConstraintAffine {
            lb: lb.clone(),
            ub: ub.clone(),
            transform: DMatrix::identity(6, 6),
        };

        let n_steps = 2;
        let (expanded_lb, constraint_mat, expanded_ub) =
            transform.expand_state::<MockPhysicsSim>(n_steps).unwrap();

        assert_eq!(expanded_lb.len(), lb.len() * n_steps);
        assert_eq!(expanded_ub.len(), ub.len() * n_steps);
        assert_eq!(constraint_mat.nrows(), lb.len() * n_steps);
    }

    #[test]
    fn test_new_single_bound_input() {
        let limit = (0.0, 1.0);
        let idx = 1;

        let transform =
            ConstraintAffine::new_single_bound_input::<MockPhysicsSim>(limit, idx).unwrap();

        let input_dim = ControllerInput::<MockPhysicsSim>::dim_q();
        assert_eq!(transform.lb.len(), 1);
        assert_eq!(transform.ub.len(), 1);
        assert_eq!(transform.lb[0], limit.0);
        assert_eq!(transform.ub[0], limit.1);

        for j in 0..input_dim {
            if j != idx {
                assert_eq!(transform.transform[(0, j)], 0.0);
            } else {
                assert_eq!(transform.transform[(0, j)], 1.0);
            }
        }
    }

    #[test]
    fn test_new_single_bound_state() {
        let limit = (-1.0, 1.0);
        let idx = 2;

        let transform =
            ConstraintAffine::new_single_bound_state::<MockPhysicsSim>(limit, idx).unwrap();

        let state_dim =
            ControllerState::<MockPhysicsSim>::dim_q() + ControllerState::<MockPhysicsSim>::dim_v();
        assert_eq!(transform.lb.len(), 1);
        assert_eq!(transform.ub.len(), 1);
        assert_eq!(transform.lb[0], limit.0);
        assert_eq!(transform.ub[0], limit.1);

        for j in 0..state_dim {
            if j != idx {
                assert_eq!(transform.transform[(0, j)], 0.0);
            } else {
                assert_eq!(transform.transform[(0, j)], 1.0);
            }
        }
    }
}
