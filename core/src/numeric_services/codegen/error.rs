use std::fmt;

#[derive(Debug)]
pub enum CodegenError {
    NotFound,
    ParseError,
    Other(String),
}

impl fmt::Display for CodegenError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CodegenError::NotFound => write!(f, "File not found"),
            CodegenError::ParseError => write!(f, "Error parsing file"),
            CodegenError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}
