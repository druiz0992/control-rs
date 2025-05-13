use crate::common::Labelizable;
use crate::numeric_services::symbolic::{SymbolicError, SymbolicEvalResult, TryIntoEvalResult};
use std::ops::{Add, Div, Mul, Sub};

/// A trait representing a state in a physical system, providing methods for
/// converting the state to and from a vector representation, as well as
/// retrieving labels for the components of the state.
///
/// # Required Methods
///
/// - `as_vec`: Converts the state into a `Vec<f64>` representation.
/// - `from_vec`: Constructs a state from a `Vec<f64>` representation.
/// - `labels`: Returns a static slice of string slices representing the labels
///   for each component of the state.
///
/// # Example
///
/// Implementing the `State` trait for a custom struct:
///
/// ```rust
/// use control_rs::physics::traits::State;
/// use macros::{StateOps,LabelOps};
/// use control_rs::common::Labelizable;
///
/// #[derive(Clone, Debug, StateOps, LabelOps)]
/// struct MyState {
///     position: f64,
///     velocity: f64,
/// }
/// ```
pub trait State:
    Labelizable
    + std::fmt::Debug
    + Send
    + Sync
    + Sized
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<f64, Output = Self>
    + Div<f64, Output = Self>
    + PartialEq
{
    fn as_vec(&self) -> Vec<f64>;
    fn from_vec(v: Vec<f64>) -> Self;

    fn get_q(&self) -> Vec<f64>;
    fn get_v(&self) -> Vec<f64>;
    fn dim_q() -> usize;
    fn dim_v() -> usize;
}

impl<S: State> TryIntoEvalResult<S> for Result<SymbolicEvalResult, SymbolicError> {
    fn try_into_eval_result(self) -> Result<S, SymbolicError> {
        match self {
            Ok(SymbolicEvalResult::Vector(expr)) => {
                let vec: Vec<f64> = expr.iter().cloned().collect();
                Ok(S::from_vec(vec))
            }
            Ok(_) => Err(SymbolicError::Other("Expected vector".into())),
            Err(e) => Err(SymbolicError::Other(e.to_string())),
        }
    }
}
