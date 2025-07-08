pub mod generic;
pub use generic::{GenericCost, GenericCostOptions};

use crate::physics::{ModelError, traits::State};
use nalgebra::{DMatrix, DVector};

pub trait CostFunction {
    type State: State;
    type Input: State;

    fn total_cost(&self, state: &[Self::State], input: &[Self::Input]) -> Result<f64, ModelError> {
        if input.len() + 1 != state.len() {
            return Err(ModelError::ConfigError(
                "State trajectory needs to have one more element than input trajectory".into(),
            ));
        }

        let mut cost = 0.0;
        for k in 0..input.len() {
            let (state_cost, input_cost) = self.stage_cost(state, input, k)?;
            cost += state_cost + input_cost;
        }
        cost += self.terminal_cost(state)?;

        Ok(0.5 * cost)
    }

    fn stage_cost(
        &self,
        state: &[Self::State],
        input: &[Self::Input],
        idx: usize,
    ) -> Result<(f64, f64), ModelError>;
    fn terminal_cost(&self, state: &[Self::State]) -> Result<f64, ModelError>;

    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64>;
    fn stage_cost_gradient(
        &self,
        state: &Self::State,
        input: &Self::Input,
        state_idx: usize,
    ) -> Result<(DVector<f64>, DVector<f64>), ModelError>;

    fn get_q(&self) -> Option<&DMatrix<f64>>;
    fn get_qn(&self) -> Option<&DMatrix<f64>>;
    fn get_r(&self) -> Option<&DMatrix<f64>>;

    fn update_q(&mut self, q: DMatrix<f64>) -> Result<(), ModelError>;
    fn update_qn(&mut self, qn: DMatrix<f64>) -> Result<(), ModelError>;
    fn update_r(&mut self, r: DMatrix<f64>) -> Result<(), ModelError>;
}
