#[derive(Debug)]
pub enum ModelError {
    Symbolic(String),
    EvaluationError,
    Unexpected(String),
    IncompleteConfiguration(String),
    SolverError(String),
    ConfigError(String),
    Other(String),
}
