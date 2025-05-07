use super::error::SymbolicError;
use super::models::SymbolicFn;
use std::sync::Arc;

/// A trait that defines a registry for managing symbolic expressions.
/// This registry is responsible for storing and retrieving symbolic expressions.
///
/// # Associated Types
/// - `Record`: The type of the records stored in the registry.
///
/// # Required Methods
/// - `get(&self, expr: &str) -> Result<Self::Record, SymbolicError>`:
///   Retrieves a record from the registry based on the given expression string.
///   Returns a `Result` containing the record or a `SymbolicError` if the expression is not found.
/// - `insert(&self, name: &str, expr: Self::Record)`:
///   Inserts a new record into the registry with the given name and expression.
pub trait SymbolicRegistry: Send + Sync + std::fmt::Debug {
    type Record;

    fn get(&self, expr: &str) -> Result<Self::Record, SymbolicError>;
    fn insert(&self, name: &str, expr: Self::Record);
}

/// A trait representing a symbolic expression that can be evaluated or converted into a function.
/// This trait is designed to work with a specific `SymbolicRegistry` implementation.
///
/// # Type Parameters
/// - `R`: The type of the registry implementing the `SymbolicRegistry` trait.
///
/// # Required Methods
/// - `clone_box(&self) -> Box<dyn SymbolicExpr<R>>`:
///   Creates a boxed clone of the current symbolic expression.
/// - `to_fn(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError>`:
///   Converts the symbolic expression into a `SymbolicFn` using the provided registry.
///   Returns a `Result` containing the function or a `SymbolicError` if the conversion fails.
pub trait SymbolicExpr<R>: Send + Sync + std::fmt::Debug + 'static
where
    R: SymbolicRegistry,
{
    fn clone_box(&self) -> Box<dyn SymbolicExpr<R>>;
    fn to_fn(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError>;
}

/// Implements the `Clone` trait for boxed `SymbolicExpr` objects.
/// This allows cloning of boxed symbolic expressions by delegating to the `clone_box` method.
///
/// # Type Parameters
/// - `R`: The type of the registry implementing the `SymbolicRegistry` trait.
impl<R> Clone for Box<dyn SymbolicExpr<R>>
where
    R: SymbolicRegistry + 'static,
{
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

pub trait TryIntoEvalResult<T> {
    fn try_into_eval_result(self) -> Result<T, SymbolicError>;
}
