use crate::utils::Labelizable;
use crate::physics::traits::State;
use macros::{LabelOps, StateOps};

#[derive(Clone, Debug, StateOps, LabelOps)]
pub struct NoInput {
    pub _dummy: f64,
}
