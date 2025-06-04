pub mod generic;
pub mod terminal_minimum_control;
use nalgebra::{DMatrix, DVector};

use crate::physics::{ModelError, traits::State};

pub trait CostFunction {
    type State: State;
    type Input: State;

    fn cost(&self, state: &[Self::State], input: &DMatrix<f64>) -> Result<f64, ModelError>;

    fn terminal_cost_gradient(&self, state: &Self::State) -> DVector<f64>;
    fn stage_cost_gradient(
        &self,
        state: &Self::State,
        state_idx: usize,
    ) -> Result<DVector<f64>, ModelError>;

    fn get_q(&self) -> Option<&DMatrix<f64>>;
    fn get_qn(&self) -> Option<&DMatrix<f64>>;
    fn get_r(&self) -> Option<&DMatrix<f64>>;

    fn update_q(&mut self, q: DMatrix<f64>) -> Result<(), ModelError>;
    fn update_qn(&mut self, qn: DMatrix<f64>) -> Result<(), ModelError>;
    fn update_r(&mut self, r: DMatrix<f64>) -> Result<(), ModelError>;
}
