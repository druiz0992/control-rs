use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::numeric_services::symbolic::{ExprScalar, ExprVector};
use control_rs::solver::RootSolver;
use control_rs::solver::models::{LineSeachConfig, OptimizerConfig};
use control_rs::solver::newton::NewtonSolver;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

/// Cost = 0.5 * x'Qx + q'x + exp(-1.3 * x[0] + 0.3 * x[1]^2)
fn get_cost_expr(unknown_expr: &ExprVector) -> ExprScalar {
    let q = [2.0, -3.0];
    let q_matrix = vec![vec![1.65539, 2.89376], vec![2.89376, 6.51521]];

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new("-1.3 * x1 + 0.3 * x2 ^ 2").exp();
    cost.add(&nonlinear).wrap()
}

fn grad_cost_norm(x: &[f64]) -> f64 {
    let x = DVector::from_vec(x.to_vec());
    let q = DVector::from_vec(vec![2.0, -3.0]);
    let q_matrix = DMatrix::from_row_slice(2, 2, &[1.65539, 2.89376, 2.89376, 6.51521]);

    let linear_grad = &q_matrix * &x + q;

    let t = -1.3 * x[0] + 0.3 * x[1].powi(2);
    let exp_term = f64::exp(t);
    let nonlinear_grad = DVector::from_vec(vec![-1.3 * exp_term, 2.0 * 0.3 * x[1] * exp_term]);

    (linear_grad + nonlinear_grad).norm()
}

fn eval_constraint(x: &[f64]) -> f64 {
    (x[0] * x[0] + x[1] * x[1]).powf(0.5) - 0.5
}

fn get_eq_constraints_expr(unknown_expr: &ExprVector) -> ExprVector {
    let constraint = unknown_expr
        .norm2()
        .unwrap()
        .sub(&ExprScalar::new("0.5"))
        .wrap();
    ExprVector::from_vec(vec![constraint])
}

#[test]
fn test_newton_root_finding() {
    let expr = ExprVector::new(&["sin(x1)", "cos(x2)"]);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-1.742410372590328, 1.4020334125022704];

    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let solver = NewtonSolver::new_root_solver(&expr, &unknown_expr, &registry, None).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    assert!(result[0].sin().abs() < 1e-6);
    assert!(result[1].cos().abs() < 1e-6);
}

#[test]
fn test_newton_root_finding_norm1_line_search() {
    let expr = ExprVector::new(&["sin(x1)", "cos(x2)"]);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-1.742410372590328, 1.4020334125022704];

    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let mut ls_opts = LineSeachConfig::default();
    ls_opts.set_merit(expr.norm1().unwrap());

    let mut options = OptimizerConfig::default();
    options.set_line_search_opts(ls_opts);

    let solver =
        NewtonSolver::new_root_solver(&expr, &unknown_expr, &registry, Some(options)).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    assert!(result[0].sin().abs() < 1e-6);
    assert!(result[1].cos().abs() < 1e-6);
}

#[test]
fn test_newton_minimization_no_constraints() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];

    let solver =
        NewtonSolver::new_minimization(&cost, None, &unknown_expr, &registry, None).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    let grad_cost_norm = grad_cost_norm(&result);
    assert!(grad_cost_norm < 1e-6);
}

#[test]
fn test_gauss_newton_minimization_no_constraints() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let mut options = OptimizerConfig::default();
    options.set_gauss_newton(true);

    let solver =
        NewtonSolver::new_minimization(&cost, None, &unknown_expr, &registry, Some(options))
            .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    let grad_cost_norm = grad_cost_norm(&result);
    assert!(grad_cost_norm < 1e-6);
}

#[test]
fn test_newton_minimization() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5, 0.0];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);

    let solver = NewtonSolver::new_minimization(
        &cost,
        Some(eq_constraints_expr),
        &unknown_expr,
        &registry,
        None,
    )
    .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    let constraint_eval = eval_constraint(&result);

    assert!(constraint_eval < 1e-6);
}

#[test]
fn test_gauss_newton_minimization() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5, 0.0];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);

    let mut options = OptimizerConfig::default();
    options.set_gauss_newton(true);

    let solver = NewtonSolver::new_minimization(
        &cost,
        Some(eq_constraints_expr),
        &unknown_expr,
        &registry,
        Some(options),
    )
    .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    let constraint_eval = eval_constraint(&result);

    assert!(constraint_eval < 1e-6);
}
