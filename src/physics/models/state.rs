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
/// use control_rs::physics::state::State;
///
/// struct MyState {
///     position: f64,
///     velocity: f64,
/// }
///
/// impl State for MyState {
///     fn as_vec(&self) -> Vec<f64> {
///         vec![self.position, self.velocity]
///     }
///
///     fn from_vec(v: Vec<f64>) -> Self {
///         Self {
///             position: v[0],
///             velocity: v[1],
///         }
///     }
///
///     fn labels() -> &'static [&'static str] {
///         &["position", "velocity"]
///     }
/// }
/// ```
/// State representation
pub trait State:
    Sized + Add<Output = Self> + Sub<Output = Self> + Mul<f64, Output = Self> + Div<f64, Output = Self>
{
    fn as_vec(&self) -> Vec<f64>;
    fn from_vec(v: Vec<f64>) -> Self;
    fn labels() -> &'static [&'static str];
}
