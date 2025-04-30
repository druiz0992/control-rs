use std::fmt;

#[derive(Debug)]
pub enum DerivativeError {
    NotFound,
    ParseError,
    Other(String),
}

impl fmt::Display for DerivativeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DerivativeError::NotFound => write!(f, "Derivative not found"),
            DerivativeError::ParseError => write!(f, "Error parsing derivative"),
            DerivativeError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}