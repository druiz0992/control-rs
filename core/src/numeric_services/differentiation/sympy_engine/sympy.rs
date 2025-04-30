use crate::numeric_services::differentiation::engine::DerivativeEngine;
use crate::numeric_services::differentiation::error::DerivativeError;
use crate::numeric_services::differentiation::models::{DerivativeRequest, DerivativeResponse};
use serde_json::json;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

/// # Constants
/// - `PYTHON_SCRIPT_PATH`: Path to the Python script that performs the differentiation.
/// - `PYTHON_INTERPRETER`: The Python interpreter to use (default is `python3`).
const PYTHON_SCRIPT_PATH: &str = "src/numeric_services/differentiation/sympy_engine/backend.py";
const PYTHON_INTERPRETER: &str = "python3";

/// The `Sympy` struct provides an implementation of the `DerivativeEngine` trait
/// for computing derivatives using a Python backend powered by the SymPy library.
#[derive(Debug, Default)]
pub struct Sympy {
    path: String,
}

impl Sympy {
    pub fn new() -> Self {
        Self {
            path: PYTHON_INTERPRETER.to_string(),
        }
    }

    /// Changes python executable path
    pub fn set_python_path(&mut self, path: &str) {
        self.path = path.to_string();
    }
}

impl DerivativeEngine for Sympy {
    fn compute_derivatives(
        &self,
        req: &DerivativeRequest,
    ) -> Result<DerivativeResponse, DerivativeError> {
        // Set up the Python process
        let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR")); // project root
        if !path.exists() {
            return Err(DerivativeError::NotFound);
        }
        path.push(PYTHON_SCRIPT_PATH);

        let mut child = Command::new(&self.path)
            .arg(path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|_| DerivativeError::Other("Failed to start Python process".to_string()))?;

        // Prepare the input JSON data to send the function description
        let input_data = json!({
            "function": req.functions,
            "variables": req.variables,
            "derivatives": req.derivatives,
        });

        // Send the input data to Python via stdin
        if let Some(stdin) = child.stdin.as_mut() {
            stdin
                .write_all(input_data.to_string().as_bytes())
                .map_err(|_| {
                    DerivativeError::Other("Failed to send input data to Python".to_string())
                })?;
        }

        let output = child
            .wait_with_output()
            .map_err(|_| DerivativeError::Other("Failed to read Python output".to_string()))?;

        let raw_response: DerivativeResponse =
            serde_json::from_str(&String::from_utf8_lossy(&output.stdout))
                .map_err(|_| DerivativeError::ParseError)?;

        let gradient = convert_python_expr_vec(&raw_response.gradient.unwrap_or_default());
        let jacobian = convert_python_expr_matrix(&raw_response.jacobian.unwrap_or_default());
        let hessian = convert_python_expr_matrix(&raw_response.hessian.unwrap_or_default());

        let response = DerivativeResponse {
            gradient: Some(gradient),
            jacobian: Some(jacobian),
            hessian: Some(hessian),
        };
        Ok(response)
    }
}

///  Converts Python-style expressions (e.g., `**`)
///   to  rust format (e.g., `^`).
fn convert_python_expr(expr: &str) -> String {
    expr.replace("**", "^")
}

/// Converts a vector of Python-style
///   expressions to rust format
fn convert_python_expr_vec(exprs: &[String]) -> Vec<String> {
    exprs.iter().map(|e| convert_python_expr(e)).collect()
}

/// Converts a matrix of Python-style
///   expressions to rust format
fn convert_python_expr_matrix(exprs: &[Vec<String>]) -> Vec<Vec<String>> {
    exprs
        .iter()
        .map(|row| convert_python_expr_vec(row))
        .collect()
}

#[cfg(test)]
mod tests {
    use crate::numeric_services::differentiation::models::DerivativeType;

    use super::*;

    #[test]
    fn test_compute_derivatives_single_f() {
        let sympy = Sympy::new();
        let variables = vec!["x".to_string(), "y".to_string()];

        let req = DerivativeRequest {
            functions: vec!["x**2 + y**2".to_string()],
            variables,
            derivatives: vec![
                DerivativeType::Gradient,
                DerivativeType::Jacobian,
                DerivativeType::Hessian,
            ],
        };

        let response = sympy.compute_derivatives(&req);
        assert!(response.is_ok());

        let response = response.unwrap();
        assert_eq!(response.gradient.unwrap(), vec!["2*x", "2*y"]);
        assert_eq!(response.jacobian.unwrap(), vec![Vec::<String>::new()]);
        assert_eq!(
            response.hessian.unwrap(),
            vec![
                vec!["2".to_string(), "0".to_string()],
                vec!["0".to_string(), "2".to_string()]
            ]
        );
    }

    #[test]
    fn test_compute_derivatives_single_f_with_parameters() {
        let sympy = Sympy::new();
        let req = DerivativeRequest {
            functions: vec!["x**2+a + y**2*u+b * t".to_string()],
            variables: vec!["x".to_string(), "y".to_string()],
            derivatives: vec![
                DerivativeType::Gradient,
                DerivativeType::Jacobian,
                DerivativeType::Hessian,
            ],
        };

        let response = sympy.compute_derivatives(&req);
        assert!(response.is_ok());

        let response = response.unwrap();
        assert_eq!(response.gradient.unwrap(), vec!["2*x", "2*u*y"]);
        assert_eq!(response.jacobian.unwrap(), vec![Vec::<String>::new()]);
        assert_eq!(
            response.hessian.unwrap(),
            vec![
                vec!["2".to_string(), "0".to_string()],
                vec!["0".to_string(), "2*u".to_string()]
            ]
        );
    }

    #[test]
    fn test_compute_derivatives_multiple_f() {
        let sympy = Sympy::new();
        let req = DerivativeRequest {
            functions: vec![
                "x**2 + y**2".to_string(),
                "cos(x)**2 + sin(y)**2".to_string(),
            ],
            variables: vec!["x".to_string(), "y".to_string()],
            derivatives: vec![
                DerivativeType::Gradient,
                DerivativeType::Jacobian,
                DerivativeType::Hessian,
            ],
        };

        let response = sympy.compute_derivatives(&req).unwrap();

        assert_eq!(response.gradient.unwrap(), Vec::<String>::new());
        assert_eq!(
            response.jacobian.unwrap(),
            vec![
                vec!["2*x".to_string(), "2*y".to_string()],
                vec![
                    "-2*sin(x)*cos(x)".to_string(),
                    "2*sin(y)*cos(y)".to_string()
                ]
            ]
        );
        assert_eq!(response.hessian.unwrap(), vec![Vec::<String>::new()]);
    }
}
