use crate::physics::traits::State;
use crate::utils::Labelizable;
use macros::{LabelOps, StateOps};

#[derive(Clone, Debug, StateOps, LabelOps)]
pub struct Quadrotor2DState {
    pub pos_x: f64,
    pub pos_y: f64,
    pub theta: f64,
    pub v_x: f64,
    pub v_y: f64,
    pub omega: f64,
}
