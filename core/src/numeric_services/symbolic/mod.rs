pub mod error;
pub mod fasteval;
pub mod models;
pub mod ports;

pub use error::*;
pub use models::*;
pub use ports::*;

pub use fasteval::matrix::ExprMatrix;
pub use fasteval::record::ExprRecord;
pub use fasteval::registry::ExprRegistry;
pub use fasteval::scalar::ExprScalar;
pub use fasteval::vector::ExprVector;
pub use ports::TryIntoEvalResult;
