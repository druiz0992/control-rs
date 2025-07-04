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

        if let (Some(state_traj), Some(input_traj)) =
            (&options.state_traj_ref, &options.input_traj_ref)
        {
            if state_traj.len() != input_traj.len() + 1 {
                return Err(ModelError::ConfigError(
                    "State trajectory should have one more element than Input trajectory".into(),
                ));
            }
        }

        if options.linear_term_flag
            && (options.state_traj_ref.is_some() || options.input_traj_ref.is_some())
        {
            return Err(ModelError::ConfigError(
                "Can't provide reference state or input if linear term flag is enabled".into(),
            ));
        }

        let state_traj_ref_vec = options
            .state_traj_ref
            .map(|traj| traj.iter().map(|s| s.to_vector()).collect());

        let input_traj_ref_vec = options
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

    fn cost_term(diff: &DVector<f64>, weight: &DMatrix<f64>) -> f64 {
        (diff.transpose() * (weight * diff))[0]
    }
}

impl<S, I> CostFunction for GenericCost<S, I>
where
    S: State,
    I: State,
{
    type State = S;
    type Input = I;

    fn stage_cost(&self, state: &[S], input: &[I], idx: usize) -> Result<(f64, f64), ModelError> {
        if idx >= state.len() || idx >= input.len() {
            return Err(ModelError::ConfigError(format!(
                "Cannot access element {} in state/input",
                idx
            )));
        }

        let state_vec = state[idx].to_vector();
        let ref_state = self
            .state_traj_ref_vec
            .as_ref()
            .map(|v| &v[idx])
            .cloned()
            .unwrap_or_else(|| DVector::zeros(state_vec.len()));

        let input_vec = input[idx].to_vector();
        let ref_input = self
            .input_traj_ref_vec
            .as_ref()
            .map(|v| &v[idx])
            .cloned()
            .unwrap_or_else(|| DVector::zeros(input_vec.len()));

        let state_cost = Self::cost_term(&(state_vec - ref_state), &self.q_matrix);
        let input_cost = Self::cost_term(&(input_vec - ref_input), &self.r_matrix);

        Ok((state_cost, input_cost))
    }

    fn terminal_cost(&self, state: &[S]) -> Result<f64, ModelError> {
        let final_state = state
            .last()
            .expect("state vec should never be empty")
            .to_vector();

        let ref_state = self
            .state_traj_ref_vec
            .as_ref()
            .and_then(|v| v.last())
            .cloned()
            .unwrap_or_else(|| DVector::zeros(final_state.len()));

        Ok(Self::cost_term(&(final_state - ref_state), &self.qn_matrix))
    }

    fn stage_cost_gradient(
        &self,
        state: &S,
        input: &I,
        idx: usize,
    ) -> Result<(DVector<f64>, DVector<f64>), ModelError> {
        let state_vec = state.to_vector();
        let ref_state = self
            .state_traj_ref_vec
            .as_ref()
            .and_then(|v| v.get(idx))
            .cloned()
            .unwrap_or_else(|| DVector::zeros(state_vec.len()));

        let input_vec = input.to_vector();
        let ref_input = self
            .input_traj_ref_vec
            .as_ref()
            .and_then(|v| v.get(idx))
            .cloned()
            .unwrap_or_else(|| DVector::zeros(input_vec.len()));

        Ok((
            &self.q_matrix * (state_vec - ref_state),
            &self.r_matrix * (input_vec - ref_input),
        ))
    }

    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64> {
        let state_vec = state.to_vector();
        let ref_state = self
            .state_traj_ref_vec
            .as_ref()
            .and_then(|v| v.last())
            .cloned()
            .unwrap_or_else(|| DVector::zeros(state_vec.len()));
        &self.qn_matrix * (state_vec - ref_state)
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
        if q.shape() != self.q_matrix.shape() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect Q Dimensions. Expecting {:?}, Obtained {:?}",
                self.q_matrix.shape(),
                q.shape()
            )));
        }
        self.q_matrix = q;
        Ok(())
    }

    fn update_qn(&mut self, qn: DMatrix<f64>) -> Result<(), ModelError> {
        if qn.shape() != self.qn_matrix.shape() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect Qn Dimensions. Expecting {:?}, Obtained {:?}",
                self.qn_matrix.shape(),
                qn.shape()
            )));
        }
        self.qn_matrix = qn;
        Ok(())
    }

    fn update_r(&mut self, r: DMatrix<f64>) -> Result<(), ModelError> {
        if r.shape() != self.r_matrix.shape() {
            return Err(ModelError::ConfigError(format!(
                "Incorrect R Dimensions. Expecting {:?}, Obtained {:?}",
                self.r_matrix.shape(),
                r.shape()
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
        let inputs = vec![MockInput::new(0.1)];

        let result = cost.total_cost(&states, &inputs);
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
        let inputs = vec![MockInput::new(0.1), MockInput::new(0.3)];

        let result = cost.total_cost(&states, &inputs);
        assert!(result.is_err());
    }
}
