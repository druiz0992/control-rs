#[derive(Debug)]
pub enum ModelError {
    Symbolic(String),
    EvaluationError,
}
