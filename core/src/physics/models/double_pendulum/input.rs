use crate::utils::Labelizable;
use crate::physics::traits::State;
use macros::{LabelOps, StateOps};

#[derive(Clone, Debug, StateOps, LabelOps)]
pub struct DoublePendulumInput {
    pub u1: f64,
    pub u2: f64,
}
