use std::fmt;

/// Error representation
#[derive(Debug, Clone)]
pub enum SymbolicError {
    ParseError,
    ExprNotFound(String),
    EvaluationError,
    UnexpectedResultType,
    Other(String),
}

impl fmt::Display for SymbolicError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SymbolicError::ParseError => write!(f, "Parse error occurred"),
            SymbolicError::ExprNotFound(expr) => write!(f, "Expression {} not found", expr),
            SymbolicError::EvaluationError => write!(f, "Evaluation error occurred"),
            SymbolicError::UnexpectedResultType => write!(f, "Unexpected result type"),
            SymbolicError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}
