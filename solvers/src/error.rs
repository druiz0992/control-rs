use std::fmt;

use symbolic_services::symbolic::SymbolicError;

#[derive(Clone, Debug)]
pub enum SolverError {
    EvaluationError,
    ConfigError(String),
    Other(String),
    Unexpected(String),
}

impl fmt::Display for SolverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolverError::EvaluationError => write!(f, "Evaluation error occurred"),
            SolverError::Unexpected(msg) => write!(f, "Unexpected error {}", msg),
            SolverError::ConfigError(msg) => write!(f, "Config error {}", msg),
            SolverError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}

impl From<SymbolicError> for SolverError {
    fn from(value: SymbolicError) -> Self {
        match value {
            SymbolicError::ParseError => SolverError::Other("Parse error".into()),
            SymbolicError::ExprNotFound(msg) => {
                SolverError::Other(format!("Expression {} not found", msg))
            }
            SymbolicError::EvaluationError => SolverError::EvaluationError,
            SymbolicError::UnexpectedResultType => {
                SolverError::Unexpected("Unexpected result type".into())
            }
            SymbolicError::IoError(msg) => SolverError::ConfigError(msg.to_string()),
            SymbolicError::Other(msg) => SolverError::Other(msg.to_string()),
        }
    }
}
