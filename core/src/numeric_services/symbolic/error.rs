/// Error representation
#[derive(Debug, Clone)]
pub enum SymbolicError {
    ParseError,
    ExprNotFound,
    EvaluationError,
    UnexpectedResultType,
    Other(String),
}
