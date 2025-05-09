use crate::physics::traits::State;
use macros::StateOps;

#[derive(Clone, Debug, StateOps)]
pub struct SlidingBrickState {
    pub pos_x: f64,
    pub pos_y: f64,
    pub v_x: f64,
    pub v_y: f64,
}
