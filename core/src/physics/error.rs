use crate::numeric_services::symbolic::SymbolicError;

#[derive(Debug)]
pub enum ModelError {
    Symbolic(String),
    EvaluationError,
    Unexpected(String),
    IncompleteConfiguration(String),
    SolverError(String),
    ConfigError(String),
    DiscretizerError(String),
    Other(String),
}

impl From<SymbolicError> for ModelError {
    fn from(value: SymbolicError) -> Self {
        match value {
            SymbolicError::ParseError => ModelError::Symbolic("Parse error".into()),
            SymbolicError::ExprNotFound(e) => ModelError::Symbolic(e),
            SymbolicError::EvaluationError => ModelError::EvaluationError,
            SymbolicError::UnexpectedResultType => {
                ModelError::Unexpected("Unexpected result type".into())
            }
            SymbolicError::IoError(e) => ModelError::Other(e),
            SymbolicError::Other(e) => ModelError::Other(e),
        }
    }
}
