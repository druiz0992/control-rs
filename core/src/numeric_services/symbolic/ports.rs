use super::error::SymbolicError;
use super::models::SymbolicFn;
use std::sync::Arc;

/// Definition of object that manages symbolic expression storage
pub trait SymbolicRegistry: Send + Sync + std::fmt::Debug {
    type Record;

    fn get(&self, expr: &str) -> Result<Self::Record, SymbolicError>;
    fn insert(&self, name: &str, expr: Self::Record);
}

pub trait SymbolicExpr<R>: Send + Sync + std::fmt::Debug + 'static
where
    R: SymbolicRegistry,
{
    fn clone_box(&self) -> Box<dyn SymbolicExpr<R>>;
    fn to_fn<'a>(&self, registry: &Arc<R>) -> Result<SymbolicFn, SymbolicError>;
}

impl<R> Clone for Box<dyn SymbolicExpr<R>>
where
    R: SymbolicRegistry + 'static,
{
    fn clone(&self) -> Self {
        self.clone_box()
    }
}
