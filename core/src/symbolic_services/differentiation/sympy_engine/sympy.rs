use crate::symbolic_services::differentiation::DerivativeType;
use crate::symbolic_services::differentiation::dtos::{DerivativeRequest, DerivativeResponse};
use crate::symbolic_services::differentiation::engine::DerivativeEngine;
use crate::symbolic_services::differentiation::error::DerivativeError;
use crate::symbolic_services::python_client::PythonClient;
use rayon::prelude::*;
use serde_json::json;

/// # Constants
/// - `PYTHON_SCRIPT_PATH`: Path to the Python script that performs the differentiation.
const PYTHON_SCRIPT_PATH: &str = "src/symbolic_services/differentiation/sympy_engine/backend.py";

type GradientChunk = Vec<String>;
type JacobianChunk = Vec<Vec<String>>;
type HessianChunk = Vec<Vec<String>>;

/// The `Sympy` struct provides an implementation of the `DerivativeEngine` trait
/// for computing derivatives using a Python backend powered by the SymPy library.
#[derive(Debug, Default)]
pub struct Sympy(PythonClient);

impl Sympy {
    pub fn new() -> Self {
        Self(PythonClient::new())
    }

    /// Changes python executable path
    pub fn set_python_path(&mut self, path: &str) {
        self.0.set_python_path(path)
    }

    pub fn compute_derivative_chunk(
        &self,
        req: &DerivativeRequest,
        pairs: &[(&String, &String)],
    ) -> Result<Vec<(GradientChunk, JacobianChunk, HessianChunk)>, DerivativeError> {
        pairs
            .par_iter()
            .map(|(function, variable)| {
                // Resolve script path
                let mut path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
                if !path.exists() {
                    return Err(DerivativeError::NotFound);
                }
                path.push(PYTHON_SCRIPT_PATH);

                let input_data = json!({
                    "function": vec![function],
                    "variables": vec![variable],
                    "derivatives": req.derivatives,
                });

                let response = self
                    .0
                    .run_python_json(path.to_str().unwrap(), &input_data)
                    .map_err(DerivativeError::Other)?;

                let raw_response: DerivativeResponse =
                    serde_json::from_str(&response).map_err(|_| DerivativeError::ParseError)?;

                let gradient = convert_python_expr_vec(&raw_response.gradient.unwrap_or_default());
                let jacobian =
                    convert_python_expr_matrix(&raw_response.jacobian.unwrap_or_default());
                let hessian = convert_python_expr_matrix(&raw_response.hessian.unwrap_or_default());
                Ok((gradient, jacobian, hessian))
            })
            .collect()
    }
}

fn separate_chunks(
    chunks: Vec<(GradientChunk, JacobianChunk, HessianChunk)>,
) -> (Vec<GradientChunk>, Vec<JacobianChunk>, Vec<HessianChunk>) {
    let (gradient_chunks, jacobian_chunks, hessian_chunks): (Vec<_>, Vec<_>, Vec<_>) =
        chunks.into_iter().fold(
            (Vec::new(), Vec::new(), Vec::new()),
            |(mut g, mut j, mut h), (gradient, jacobian, hessian)| {
                g.push(gradient);
                j.push(jacobian);
                h.push(hessian);
                (g, j, h)
            },
        );
    (gradient_chunks, jacobian_chunks, hessian_chunks)
}

fn merge_gradient_chunks(chunks: Vec<Vec<String>>) -> Vec<String> {
    chunks.into_iter().flatten().collect()
}

fn merge_jacobian_chunks(chunks: Vec<Vec<Vec<String>>>, n_columns: usize) -> Vec<Vec<String>> {
    chunks
        .chunks(n_columns)
        .map(|chunk| {
            chunk
                .iter()
                .flat_map(|v| v.iter().cloned())
                .flatten()
                .collect()
        })
        .collect()
}

impl DerivativeEngine for Sympy {
    fn compute_derivatives(
        &self,
        req: &DerivativeRequest,
    ) -> Result<DerivativeResponse, DerivativeError> {
        // Generate all (function, variable) pairs
        let pairs: Vec<(&String, &String)> = req
            .functions
            .iter()
            .flat_map(|f| req.variables.iter().map(move |v| (f, v)))
            .collect();

        let mut new_derivatives_req = req.derivatives.clone();
        new_derivatives_req.retain(|d| d != &DerivativeType::Hessian);
        let contains_hessian = new_derivatives_req != req.derivatives;
        let contains_gradient_jacobian = req.derivatives.contains(&DerivativeType::Gradient)
            || req.derivatives.contains(&DerivativeType::Jacobian);

        let (gradient, jacobian): (Option<Vec<String>>, Option<Vec<Vec<String>>>) =
            if contains_gradient_jacobian {
                let chunks = self.compute_derivative_chunk(req, &pairs)?;
                let (gradient_chunks, jacobian_chunks, _) = separate_chunks(chunks);
                let gradient = merge_gradient_chunks(gradient_chunks);
                let jacobian = merge_jacobian_chunks(jacobian_chunks, req.variables.len());
                (Some(gradient), Some(jacobian))
            } else if contains_hessian {
                let new_gradient_req = DerivativeRequest {
                    functions: req.functions.clone(),
                    variables: req.variables.clone(),
                    derivatives: vec![DerivativeType::Gradient],
                };
                let chunks = self.compute_derivative_chunk(&new_gradient_req, &pairs)?;
                let (gradient_chunks, _, _) = separate_chunks(chunks);
                let gradient = merge_gradient_chunks(gradient_chunks);
                (Some(gradient), None)
            } else {
                (None, None)
            };

        let hessian = if contains_hessian {
            let new_jacobian_req = DerivativeRequest {
                functions: gradient.clone().unwrap(),
                variables: req.variables.clone(),
                derivatives: vec![DerivativeType::Jacobian],
            };
            let pairs: Vec<(&String, &String)> = new_jacobian_req
                .functions
                .iter()
                .flat_map(|f| req.variables.iter().map(move |v| (f, v)))
                .collect();
            let chunks = self.compute_derivative_chunk(&new_jacobian_req, &pairs)?;
            let (_, jacobian_chunks, _) = separate_chunks(chunks);
            let jacobian = merge_jacobian_chunks(jacobian_chunks, req.variables.len());
            Some(jacobian)
        } else {
            None
        };

        Ok(DerivativeResponse {
            gradient,
            jacobian,
            hessian,
        })
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
    use crate::symbolic_services::differentiation::dtos::DerivativeType;

    use super::*;

    #[test]
    fn test_compute_jacobian() {
        let sympy = Sympy::new();
        let variables = vec!["x".to_string(), "y".to_string()];

        let req = DerivativeRequest {
            functions: vec!["2*x".to_string(), "2*y".to_string()],
            variables,
            derivatives: vec![DerivativeType::Jacobian],
        };

        let response = sympy.compute_derivatives(&req);
        assert!(response.is_ok());

        let response = response.unwrap();
        assert_eq!(response.jacobian.unwrap(), vec![["2", "0"], ["0", "2"]]);
    }

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
        assert_eq!(response.jacobian.unwrap(), vec![["2*x", "2*y"]]);
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
        assert_eq!(response.jacobian.unwrap(), vec![["2*x", "2*u*y"]]);
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

        //assert_eq!(response.gradient.unwrap(), Vec::<String>::new());
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
        //assert_eq!(response.hessian.unwrap(), vec![Vec::<String>::new()]);
    }
}
