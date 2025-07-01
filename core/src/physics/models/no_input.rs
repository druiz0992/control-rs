use crate::physics::traits::State;
use crate::utils::Labelizable;
use macros::{LabelOps, StateOps};

#[derive(Clone, Debug, StateOps, LabelOps)]
pub struct NoInput {
    pub _dummy: f64,
}
