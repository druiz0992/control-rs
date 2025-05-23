use nalgebra::{DMatrix, DVector};

use super::input::LtiInput;
use super::model::LtiModel;
use super::state::LtiState;
use crate::physics::models::dynamics::LinearDynamics;
use crate::physics::traits::Dynamics;
use crate::physics::traits::State;

impl<const N: usize, const C: usize, const I: usize> Dynamics for LtiModel<N, C, I> {
    type State = LtiState<N, C>;
    type Input = LtiInput<I, 0>;

    fn dynamics(&self, state: &Self::State, input: Option<&Self::Input>) -> Self::State {
        let u = DVector::from_row_slice(&input.unwrap_or(&LtiInput::<I, 0>::default()).to_vec());
        let s =
            self.get_a_as_slice() * DVector::from_vec(state.to_vec()) + self.get_b_as_slice() * u;
        Self::State::from_slice(s.as_slice())
    }

    fn state_dims(&self) -> (usize, usize) {
        (LtiState::<N, C>::dim_q(), LtiState::<N, C>::dim_v())
    }
}

impl<const N: usize, const C: usize, const I: usize> LinearDynamics for LtiModel<N, C, I> {
    fn get_state_slice(&self) -> &DMatrix<f64> {
        self.get_a_as_slice()
    }
    fn get_control_slice(&self) -> &DMatrix<f64> {
        self.get_b_as_slice()
    }
}
