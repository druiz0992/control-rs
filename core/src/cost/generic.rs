use super::CostFunction;
use crate::physics::{ModelError, traits::State};
use nalgebra::{DMatrix, DVector};
use std::marker::PhantomData;

#[derive(Default)]
pub struct GenericCostOptions<S, I> {
    state_traj_ref: Option<Vec<S>>,
    input_traj_ref: Option<Vec<I>>,
    /// controllers using OSQP optimizer, build cost function with a linear term,
    /// which is made up from x_ref and u_ref. Therefore, if we are enabling linear_term_flag,
    /// stater_traj and input_traj will be discarded to build the cost function
    linear_term_flag: bool,
}

impl<S, I> GenericCostOptions<S, I>
where
    S: State + Clone,
    I: State + Clone,
{
    pub fn new() -> Self {
        GenericCostOptions::default()
    }

    pub fn get_reference_state_trajectory(&self) -> &Option<Vec<S>> {
        &self.state_traj_ref
    }
    pub fn get_reference_input_trajectory(&self) -> &Option<Vec<I>> {
        &self.input_traj_ref
    }
    pub fn get_linear_term_flag(&self) -> bool {
        self.linear_term_flag
    }

    pub fn set_reference_state_trajectory(self, state_traj_ref: &[S]) -> Self {
        let mut new = self;
        new.state_traj_ref = Some(state_traj_ref.to_vec());

        new
    }

    pub fn set_reference_input_trajectory(self, input_traj_ref: &[I]) -> Self {
        let mut new = self;
        new.input_traj_ref = Some(input_traj_ref.to_vec());

        new
    }

    pub fn set_linear_term(self, flag: bool) -> Self {
        let mut new = self;
        new.linear_term_flag = flag;

        new
    }
}

#[derive(Clone)]
pub struct GenericCost<S, I>
where
    S: State,
    I: State,
{
    qn_matrix: DMatrix<f64>,
    q_matrix: DMatrix<f64>,
    r_matrix: DMatrix<f64>,
    state_traj_ref_vec: Option<Vec<DVector<f64>>>,
    input_traj_ref_vec: Option<Vec<DVector<f64>>>,
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
        options: Option<GenericCostOptions<S, I>>,
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

        let options = options.unwrap_or_default();

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

        if let (Some(state_traj), Some(input_traj)) = (
            options.state_traj_ref.as_ref(),
            options.input_traj_ref.as_ref(),
        ) {
            if state_traj.len() != input_traj.len() + 1 {
                return Err(ModelError::ConfigError(
                    "State trajectory should have one more element than Input trajectory".into(),
                ));
            }
        }

        if options.state_traj_ref.as_ref().is_some() && options.linear_term_flag {
            return Err(ModelError::ConfigError(
                "Can't provide reference state if linear term flag is enabled".into(),
            ));
        }
        if options.input_traj_ref.as_ref().is_some() && options.linear_term_flag {
            return Err(ModelError::ConfigError(
                "Can't provide reference input if linear term flag is enabled".into(),
            ));
        }

        let state_traj_ref_vec: Option<Vec<DVector<f64>>> = options
            .state_traj_ref
            .map(|traj| traj.iter().map(|s| s.to_vector()).collect());

        let input_traj_ref_vec: Option<Vec<DVector<f64>>> = options
            .input_traj_ref
            .map(|traj| traj.iter().map(|s| s.to_vector()).collect());

        Ok(Self {
            qn_matrix,
            q_matrix,
            r_matrix,
            state_traj_ref_vec,
            input_traj_ref_vec,
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

        if let Some(state_traj_ref) = self.state_traj_ref_vec.as_ref() {
            if state_traj_ref.len() != state.len() {
                return Err(ModelError::ConfigError(
                    "Mismatch in state trajectory length".into(),
                ));
            }
        }

        if let Some(input_traj_ref) = self.input_traj_ref_vec.as_ref() {
            if input_traj_ref.len() != input.len() {
                return Err(ModelError::ConfigError(
                    "Mismatch in input trajectory length".into(),
                ));
            }
        }

        let input_cost: f64 = input
            .column_iter()
            .take(state.len() - 1)
            .enumerate()
            .map(|(i, v)| {
                let ref_input = if let Some(ref_vec) = self.input_traj_ref_vec.as_ref() {
                    &ref_vec[i]
                } else {
                    // Assume dimensions from current input vector
                    &DVector::zeros(v.len())
                };

                let diff = v - ref_input;
                let grad = &self.r_matrix * &diff;
                (v.transpose() * &grad)[(0, 0)]
            })
            .sum();

        let state_cost: f64 = state
            .iter()
            .take(state.len() - 1)
            .enumerate()
            .map(|(i, s)| {
                let v = s.to_vector();
                let ref_state = if let Some(ref_vec) = self.state_traj_ref_vec.as_ref() {
                    &ref_vec[i]
                } else {
                    // Assume dimensions from current input vector
                    &DVector::zeros(v.len())
                };
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

        let final_ref_state = if let Some(ref state_vec) = self.state_traj_ref_vec {
            state_vec
                .last()
                .expect("state_traj_ref_vec should not be empty when Some")
        } else {
            &DVector::zeros(final_state.len())
        };

        let diff = final_state - final_ref_state;
        let grad = &self.qn_matrix * &diff;

        let terminal_cost = (0.5 * diff.transpose() * &grad)[(0, 0)];

        Ok(staging_cost + terminal_cost)
    }

    fn stage_cost_gradient(&self, state: &S, state_idx: usize) -> Result<DVector<f64>, ModelError> {
        if let Some(state_traj_ref) = self.state_traj_ref_vec.as_ref() {
            if state_idx >= state_traj_ref.len() {
                return Err(ModelError::ConfigError(
                    "Mismatch in state trajectory length".into(),
                ));
            }
        }

        let v = state.to_vector();
        let ref_state = if let Some(ref_vec) = self.state_traj_ref_vec.as_ref() {
            &ref_vec[state_idx]
        } else {
            // Assume dimensions from current input vector
            &DVector::zeros(v.len())
        };

        let diff = &v - ref_state;
        Ok(&self.q_matrix * &diff)
    }

    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64> {
        let v = state.to_vector();
        let final_ref_state = if let Some(ref_vec) = self.state_traj_ref_vec.as_ref() {
            ref_vec
                .last()
                .expect("state_traj_vec should never be empty")
        } else {
            // Assume dimensions from current input vector
            &DVector::zeros(v.len())
        };
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

    fn update_q(&mut self, q: DMatrix<f64>) -> Result<(), ModelError> {
        if q.nrows() != self.q_matrix.nrows() || q.ncols() != self.q_matrix.ncols() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect Q Dimensions. Expecting {}, Obtained {}",
                q.nrows(),
                self.q_matrix.ncols()
            )));
        }
        self.q_matrix = q;

        Ok(())
    }
    fn update_qn(&mut self, qn: DMatrix<f64>) -> Result<(), ModelError> {
        if qn.nrows() != self.qn_matrix.nrows() || qn.ncols() != self.qn_matrix.ncols() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect Qn Dimensions. Expecting {}, Obtained {}",
                qn.nrows(),
                self.qn_matrix.ncols()
            )));
        }
        self.qn_matrix = qn;

        Ok(())
    }

    fn update_r(&mut self, r: DMatrix<f64>) -> Result<(), ModelError> {
        if r.nrows() != self.r_matrix.nrows() || r.ncols() != self.r_matrix.ncols() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect R Dimensions. Expecting {}, Obtained {}",
                r.nrows(),
                self.r_matrix.ncols()
            )));
        }
        self.r_matrix = r;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::physics::traits::State;
    use crate::utils::Labelizable;
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

        let options = GenericCostOptions::new().set_reference_state_trajectory(&state_traj);
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, Some(options));
        assert!(cost.is_ok());
    }

    #[test]
    fn test_new_invalid_q_matrix() {
        let q_matrix = DMatrix::zeros(4, 3); // Not square
        let qn_matrix = DMatrix::identity(4, 4);
        let r_matrix = DMatrix::identity(2, 2);
        let state_traj = vec![MockState::new(1.0, 2.0), MockState::new(3.0, 4.0)];

        let options = GenericCostOptions::new().set_reference_state_trajectory(&state_traj);
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, Some(options));
        assert!(cost.is_err());
    }

    #[test]
    fn test_cost_valid() {
        let q_matrix = DMatrix::identity(2, 2);
        let qn_matrix = DMatrix::identity(2, 2);
        let r_matrix = DMatrix::identity(1, 1);
        let state_traj = vec![MockState::new(1.0, 2.0), MockState::new(1.5, 2.5)];

        let options = GenericCostOptions::new().set_reference_state_trajectory(&state_traj);
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, Some(options))
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

        let options = GenericCostOptions::new().set_reference_state_trajectory(&state_traj);
        let cost =
            GenericCost::<MockState, MockInput>::new(q_matrix, qn_matrix, r_matrix, Some(options))
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
