use crate::common::Labelizable;
use crate::physics::traits::State;
use macros::{LabelOps, StateOps};

#[derive(Clone, Debug, StateOps, LabelOps)]
pub struct CartPoleInput {
    pub u1: f64,
}
