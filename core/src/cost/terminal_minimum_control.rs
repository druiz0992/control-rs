use super::CostFunction;
use crate::physics::{ModelError, traits::State};
use nalgebra::{DMatrix, DVector};
use std::marker::PhantomData;

pub struct TerminalMinControlCost<S, I> {
    qn_matrix: DMatrix<f64>,
    r_matrix: DMatrix<f64>,
    final_state_vec: DVector<f64>,
    _phantom_i: PhantomData<I>,
    _phantom_s: PhantomData<S>,
}

impl<S, I> TerminalMinControlCost<S, I>
where
    S: State,
    I: State,
{
    pub fn new(
        qn_matrix: DMatrix<f64>,
        r_matrix: DMatrix<f64>,
        final_state: S,
    ) -> Result<Self, ModelError> {
        if qn_matrix.nrows() != qn_matrix.ncols() {
            return Err(ModelError::ConfigError(
                "Q cost matrix needs to be square".into(),
            ));
        }
        if r_matrix.nrows() != r_matrix.ncols() {
            return Err(ModelError::ConfigError(
                "R matrix needs to be square".into(),
            ));
        }
        let state_dim = S::dim_q() + S::dim_v();
        let input_dim = I::dim_q() + I::dim_v();

        if qn_matrix.nrows() != state_dim {
            return Err(ModelError::ConfigError(
                "Qn matrix dimension must match state dimension".into(),
            ));
        }
        if r_matrix.nrows() != input_dim {
            return Err(ModelError::ConfigError(
                "R matrix dimension must match input dimension".into(),
            ));
        }

        Ok(Self {
            qn_matrix,
            r_matrix,
            final_state_vec: final_state.to_vector(),
            _phantom_i: PhantomData,
            _phantom_s: PhantomData,
        })
    }
}

impl<S, I> CostFunction for TerminalMinControlCost<S, I>
where
    S: State,
    I: State,
{
    type State = S;
    type Input = I;

    fn cost(&self, state: &[Self::State], input: &DMatrix<f64>) -> Result<f64, ModelError> {
        let staging_cost: f64 = input
            .column_iter()
            .map(|v| {
                let grad = &self.r_matrix * v;
                (v.transpose() * &grad)[(0, 0)]
            })
            .sum();

        let final_state = state
            .last()
            .expect("state vec should never be empty")
            .to_vector();
        let diff = &final_state - &self.final_state_vec;
        let grad = &self.qn_matrix * &diff;
        let terminal_cost = (diff.transpose() * &grad)[(0, 0)];

        Ok(0.5 * (terminal_cost + staging_cost))
    }

    fn stage_cost_gradient(
        &self,
        state: &Self::State,
        _state_idx: usize,
    ) -> Result<DVector<f64>, ModelError> {
        Ok(DVector::zeros(state.to_vec().len()))
    }
    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64> {
        let v = state.to_vector();
        let diff = &v - &self.final_state_vec;
        &self.qn_matrix * diff
    }

    fn get_q(&self) -> Option<&DMatrix<f64>> {
        None
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
    use crate::utils::Labelizable;
    use crate::physics::traits::State;
    use macros::{LabelOps, StateOps};

    #[derive(Clone, Debug, StateOps, LabelOps)]
    struct MockState {
        pub f1: f64,
        pub f2: f64,
        pub f3: f64,
        pub f4: f64,
    }

    #[derive(Clone, Debug, StateOps, LabelOps)]
    struct MockInput {
        pub u1: f64,
        pub u2: f64,
    }

    #[test]
    fn test_new_valid_matrices() {
        let qn_matrix = DMatrix::identity(4, 4);
        let r_matrix = DMatrix::identity(2, 2);
        let final_state = MockState::new(1.0, 2.0, 3.0, 4.0);

        let cost =
            TerminalMinControlCost::<MockState, MockInput>::new(qn_matrix, r_matrix, final_state);
        assert!(cost.is_ok());
    }

    #[test]
    fn test_new_invalid_qn_matrix() {
        let qn_matrix = DMatrix::identity(3, 4); // Not square
        let r_matrix = DMatrix::identity(2, 2);
        let final_state = MockState::new(1.0, 2.0, 3.0, 4.0);

        let cost =
            TerminalMinControlCost::<MockState, MockInput>::new(qn_matrix, r_matrix, final_state);
        assert!(matches!(cost, Err(ModelError::ConfigError(_))));
    }

    #[test]
    fn test_new_invalid_r_matrix() {
        let qn_matrix = DMatrix::identity(4, 4);
        let r_matrix = DMatrix::identity(3, 2); // Not square
        let final_state = MockState::new(1.0, 2.0, 3.0, 4.0);

        let cost =
            TerminalMinControlCost::<MockState, MockInput>::new(qn_matrix, r_matrix, final_state);
        assert!(matches!(cost, Err(ModelError::ConfigError(_))));
    }

    #[test]
    fn test_cost() {
        let qn_matrix = DMatrix::identity(4, 4);
        let r_matrix = DMatrix::identity(2, 2);
        let final_state = MockState::new(1.0, 2.0, 3.0, 4.0);
        let cost = TerminalMinControlCost::<MockState, MockInput>::new(
            qn_matrix,
            r_matrix,
            final_state.clone(),
        )
        .unwrap();

        let inputs = DMatrix::from_columns(&[
            MockInput::new(1.0, 0.0).to_vector(),
            MockInput::new(0.0, 1.0).to_vector(),
        ]);
        let total_cost = cost.cost(&[final_state], &inputs).unwrap();
        assert_eq!(total_cost, 1.0);
    }
}
