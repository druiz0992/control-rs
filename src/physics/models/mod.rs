pub mod state;
pub mod dynamics;
pub mod double_pendulum;

pub use double_pendulum::dynamics::DoublePendulum;
pub use double_pendulum::state::DoublePendulumState;
