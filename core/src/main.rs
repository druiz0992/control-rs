use meval::{Context, Expr};
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::collections::HashMap;
use std::io::Write;
use std::process::{Command, Stdio};

#[derive(Serialize, Deserialize, Debug)]
struct GradientHessian {
    gradient: Option<Vec<String>>,
    jacobian: Option<Vec<Vec<String>>>,
    hessian: Option<Vec<Vec<String>>>,
}

// Function to call Python and get the gradient and Hessian
fn compute_derivatives(
    function: Vec<&str>,
    variables: &[&str],
    request: &[&str],
) -> GradientHessian {
    // Set up the Python process
    let mut child = Command::new("python3")
        .arg("python_backend.py") // Path to the Python script
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .expect("Failed to start Python process");

    // Prepare the input JSON data to send the function description
    let input_data = json!({
        "function": function,
        "variables": variables,
        "request": request
    });

    // Send the input data to Python via stdin
    if let Some(stdin) = child.stdin.as_mut() {
        stdin
            .write_all(input_data.to_string().as_bytes())
            .expect("Failed to send input data to Python");
    }

    // Capture the output (the gradient and Hessian)
    let output = child
        .wait_with_output()
        .expect("Failed to read Python output");

    /*
    println!(
        "Python stdout:\n{}",
        String::from_utf8_lossy(&output.stdout)
    );
    */

    // Parse the JSON output from Python
    let gradient_hessian: GradientHessian =
        serde_json::from_str(&String::from_utf8_lossy(&output.stdout)).unwrap();

    gradient_hessian
}

fn convert_python_expr(expr: &str) -> String {
    expr.replace("**", "^")
}

fn convert_python_expr_vec(exprs: &[String]) -> Vec<String> {
    exprs.iter().map(|e| convert_python_expr(e)).collect()
}

fn convert_python_expr_matrix(exprs: &[Vec<String>]) -> Vec<Vec<String>> {
    exprs
        .iter()
        .map(|row| convert_python_expr_vec(row))
        .collect()
}

fn evaluate_expr(expr_str: &str, vars: &HashMap<&str, f64>) -> f64 {
    let expr = expr_str.parse::<Expr>().expect("Invalid expression");

    // Build context with variables
    let mut ctx = Context::new();
    for (&k, &v) in vars {
        ctx.var(k, v);
    }

    // Add standard math functions like sin, cos, etc.
    ctx.func("sin", f64::sin);
    ctx.func("cos", f64::cos);
    ctx.func("tan", f64::tan);
    ctx.func("exp", f64::exp);
    ctx.func("log", f64::ln);
    ctx.func("sqrt", f64::sqrt);

    // Add constants like pi and e
    ctx.var("pi", std::f64::consts::PI);
    ctx.var("e", std::f64::consts::E);

    expr.eval_with_context(&ctx).expect("Evaluation failed")
}

fn main() {
    // The function description sent to Python (as a string)
    let function_description = vec![
        "omega1",
        "(m2 * g * sin(theta2) * cos(theta1 - theta2) - m2 * sin(theta1 - theta2) * (l1 * cos(theta1 - theta2) * omega1**2 + l2 * omega2**2) - (m1 + m2) * g * sin(theta1)) / (l1 * (m1 + m2 * sin(theta1 - theta2)**2))",
        "omega2",
        "((m1 + m2) * (l1 * omega1**2 * sin(theta1 - theta2) - g * sin(theta2) + g * sin(theta1) * cos(theta1 - theta2)) + m2 * l2 * omega2**2 * sin(theta1 - theta2) * cos(theta1 - theta2)) / (l2 * (m1 + m2 * sin(theta1 - theta2)**2))",
    ];
    let variables = ["theta1", "omega1", "theta2", "omega2"];
    let request = ["jacobian"];

    // Get the gradient and Hessian from Python
    let result = compute_derivatives(function_description, &variables, &request);

    // Print the results (gradient and Hessian) as symbolic expressions
    println!("Gradient: {:?}", result.gradient);
    println!("Hessian: {:?}", result.hessian);

    let gradient = convert_python_expr_vec(&result.gradient.unwrap_or_default());
    let jacobian = convert_python_expr_matrix(&result.jacobian.unwrap_or_default());
    let hessian = convert_python_expr_matrix(&result.hessian.unwrap_or_default());

    // Define variable values
    let mut vars = HashMap::new();
    vars.insert("theta1", 1.0);
    vars.insert("omega1", 0.0);
    vars.insert("theta2", 1.0);
    vars.insert("omega2", 0.0);

    // parameters
    vars.insert("m1", 1.0);
    vars.insert("m2", 1.0);
    vars.insert("l1", 1.0);
    vars.insert("l2", 1.0);
    vars.insert("g", 9.81);

    // Evaluate gradient
    let grad_values: Vec<f64> = gradient
        .iter()
        .map(|expr| evaluate_expr(expr, &vars))
        .collect();

    println!("Gradient: {:?}", grad_values);

    // Evaluate jacobian
    let jacobian_values: Vec<Vec<f64>> = jacobian
        .iter()
        .map(|row| row.iter().map(|expr| evaluate_expr(expr, &vars)).collect())
        .collect();

    println!("Jacobian: {:?}", jacobian_values);

    // Evaluate hessian
    let hessian_values: Vec<Vec<f64>> = hessian
        .iter()
        .map(|row| row.iter().map(|expr| evaluate_expr(expr, &vars)).collect())
        .collect();

    println!("Hessian: {:?}", hessian_values);
}
