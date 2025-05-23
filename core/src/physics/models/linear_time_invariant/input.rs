use crate::common::Labelizable;
use crate::physics::traits::State;
use macros::{ArrayLabelOps, ArrayStateOps};

#[derive(Clone, Debug, ArrayStateOps, ArrayLabelOps)]
pub struct LtiInput<const I: usize, const C: usize> {
    pub u: [f64; I],
}
