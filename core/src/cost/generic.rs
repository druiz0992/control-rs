use super::CostFunction;
use crate::physics::{ModelError, traits::State};
use nalgebra::{DMatrix, DVector};
use std::marker::PhantomData;

#[derive(Clone)]
pub struct GenericCost<S, I>
where
    S: State,
    I: State,
{
    qn_matrix: DMatrix<f64>,
    q_matrix: DMatrix<f64>,
    r_matrix: DMatrix<f64>,
    state_traj_vec: Vec<DVector<f64>>,
    _phantom_s: PhantomData<S>,
    _phantom_i: PhantomData<I>,
}
impl<S, I> GenericCost<S, I>
where
    S: State,
    I: State,
{
    pub fn new(
        q_matrix: DMatrix<f64>,
        qn_matrix: DMatrix<f64>,
        r_matrix: DMatrix<f64>,
        state_traj: Vec<S>,
    ) -> Result<Self, ModelError> {
        if !q_matrix.is_square() {
            return Err(ModelError::ConfigError("Q matrix must be square".into()));
        }
        if !qn_matrix.is_square() {
            return Err(ModelError::ConfigError("Qn matrix must be square".into()));
        }
        if !r_matrix.is_square() {
            return Err(ModelError::ConfigError("R matrix must be square".into()));
        }
        if q_matrix.nrows() != qn_matrix.nrows() {
            return Err(ModelError::ConfigError(
                "Q and Qn matrices must have the same dimension".into(),
            ));
        }

        let state_traj_vec: Vec<_> = state_traj.iter().map(|s| s.to_vector()).collect();

        if state_traj_vec.is_empty() {
            return Err(ModelError::ConfigError(
                "State trajectory cannot be empty".into(),
            ));
        }

        let state_dim = S::dim_q() + S::dim_v();
        let input_dim = I::dim_q() + I::dim_v();

        if q_matrix.nrows() != state_dim {
            return Err(ModelError::ConfigError(
                "Q matrix dimension must match state dimension".into(),
            ));
        }
        if r_matrix.nrows() != input_dim {
            return Err(ModelError::ConfigError(
                "R matrix dimension must match input dimension".into(),
            ));
        }

        Ok(Self {
            qn_matrix,
            q_matrix,
            r_matrix,
            state_traj_vec,
            _phantom_s: PhantomData,
            _phantom_i: PhantomData,
        })
    }
}

impl<S, I> CostFunction for GenericCost<S, I>
where
    S: State,
    I: State,
{
    type State = S;
    type Input = I;

    fn cost(&self, state: &[S], input: &DMatrix<f64>) -> Result<f64, ModelError> {
        if input.ncols() + 1 != state.len() {
            return Err(ModelError::ConfigError(
                "State trajectory needs to have one more element than input trajectory".into(),
            ));
        }
        if state.len() != self.state_traj_vec.len() {
            return Err(ModelError::ConfigError(
                "Mismatch in state trajectory length".into(),
            ));
        }

        let input_cost: f64 = input
            .column_iter()
            .map(|v| {
                let grad = &self.r_matrix * v;
                (v.transpose() * &grad)[(0, 0)]
            })
            .sum();

        let state_cost: f64 = state
            .iter()
            .take(state.len() - 1)
            .enumerate()
            .map(|(i, s)| {
                let v = s.to_vector();
                let ref_state = &self.state_traj_vec[i];
                let diff = &v - ref_state;
                let grad = &self.q_matrix * &diff;
                (diff.transpose() * &grad)[(0, 0)]
            })
            .sum();
        let staging_cost = 0.5 * (state_cost + input_cost);

        let final_state = state
            .last()
            .expect("state vec should never be empty")
            .to_vector();
        let final_ref_state = self
            .state_traj_vec
            .last()
            .expect("state_traj_vec should never be empty");

        let diff = final_state - final_ref_state;
        let grad = &self.qn_matrix * &diff;

        let terminal_cost = (0.5 * diff.transpose() * &grad)[(0, 0)];

        Ok(staging_cost + terminal_cost)
    }

    fn stage_cost_gradient(&self, state: &S, state_idx: usize) -> Result<DVector<f64>, ModelError> {
        if state_idx >= self.state_traj_vec.len() {
            return Err(ModelError::ConfigError(
                "Mismatch in state trajectory length".into(),
            ));
        }

        let v = state.to_vector();
        let ref_state = &self.state_traj_vec[state_idx];
        let diff = &v - ref_state;
        Ok(&self.q_matrix * &diff)
    }

    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64> {
        let v = state.to_vector();
        let final_ref_state = self
            .state_traj_vec
            .last()
            .expect("state_traj_vec should never be empty");
        let diff = &v - final_ref_state;
        &self.qn_matrix * diff
    }

    fn get_q(&self) -> Option<&DMatrix<f64>> {
        Some(&self.q_matrix)
    }
    fn get_qn(&self) -> Option<&DMatrix<f64>> {
        Some(&self.qn_matrix)
    }
    fn get_r(&self) -> Option<&DMatrix<f64>> {
        Some(&self.r_matrix)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::common::Labelizable;
    use crate::physics::traits::State;
    use macros::{LabelOps, StateOps};

    #[derive(Clone, Debug, StateOps, LabelOps)]
    struct MockState {
        pub f1: f64,
        pub f2: f64,
    }

    #[derive(Clone, Debug, StateOps, LabelOps)]
    struct MockInput {
        pub u1: f64,
    }

    #[test]
    fn test_new_valid_inputs() {
        let q_matrix = DMatrix::identity(2, 2);
        let qn_matrix = DMatrix::identity(2, 2);
        let r_matrix = DMatrix::identity(1, 1);
        let state_traj = vec![
            MockState::new(1.0, 3.0),
            MockState::new(2.3, 4.3),
            MockState::new(4.3, 3.2),
        ];

        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, state_traj);
        assert!(cost.is_ok());
    }

    #[test]
    fn test_new_invalid_q_matrix() {
        let q_matrix = DMatrix::zeros(4, 3); // Not square
        let qn_matrix = DMatrix::identity(4, 4);
        let r_matrix = DMatrix::identity(2, 2);
        let state_traj = vec![MockState::new(1.0, 2.0), MockState::new(3.0, 4.0)];

        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, state_traj);
        assert!(cost.is_err());
    }

    #[test]
    fn test_cost_valid() {
        let q_matrix = DMatrix::identity(2, 2);
        let qn_matrix = DMatrix::identity(2, 2);
        let r_matrix = DMatrix::identity(1, 1);
        let state_traj = vec![MockState::new(1.0, 2.0), MockState::new(1.5, 2.5)];
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, state_traj)
                .unwrap();

        let states = vec![MockState::new(1.0, 2.0), MockState::new(1.5, 2.5)];
        let inputs = DMatrix::from_columns(&[MockInput::new(0.1).to_vector()]);

        let result = cost.cost(&states, &inputs);
        assert!(result.is_ok());
    }

    #[test]
    fn test_cost_invalid_length() {
        let q_matrix = DMatrix::identity(2, 2);
        let qn_matrix = DMatrix::identity(2, 2);
        let r_matrix = DMatrix::identity(1, 1);
        let state_traj = vec![MockState::new(1.0, 2.0), MockState::new(1.5, 2.5)];
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, state_traj)
                .unwrap();

        let states = vec![MockState::new(1.0, 2.0), MockState::new(3.0, 4.0)]; // Mismatch in length
        let inputs = DMatrix::from_columns(&[
            MockInput::new(0.1).to_vector(),
            MockInput::new(0.3).to_vector(),
        ]);

        let result = cost.cost(&states, &inputs);
        assert!(result.is_err());
    }
}
