use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::numeric_services::symbolic::{ExprScalar, ExprVector};
use control_rs::physics::traits::State;
use control_rs::solver::RootSolver;
use control_rs::solver::models::{LineSeachConfig, OptimizerConfig};
use control_rs::solver::newton::NewtonSolver;
use macros::StateOps;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

#[derive(StateOps, Debug)]
struct MyState {
    x1: f64,
    x2: f64,
}
#[derive(StateOps, Debug)]
struct MyState2 {
    x1: f64,
    x2: f64,
    lambda_0: f64,
}

#[test]
fn test_newton_root_finding() {
    let expr = ExprVector::new(&["sin(x1)", "cos(x2)"]);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState {
        x1: -1.742410372590328,
        x2: 1.4020334125022704,
    };

    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let solver = NewtonSolver::new_root_solver(&expr, &unknown_expr, &registry, None).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    assert!(result.x1.sin().abs() < 1e-6);
    assert!(result.x2.cos().abs() < 1e-6);
}

#[test]
fn test_newton_root_finding_norm1_line_search() {
    let expr = ExprVector::new(&["sin(x1)", "cos(x2)"]);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState {
        x1: -1.742410372590328,
        x2: 1.4020334125022704,
    };

    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let mut ls_opts = LineSeachConfig::default();
    ls_opts.set_merit(expr.norm1().unwrap());

    let mut options = OptimizerConfig::default();
    options.set_line_search_opts(ls_opts);

    let solver =
        NewtonSolver::new_root_solver(&expr, &unknown_expr, &registry, Some(options)).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();

    assert!(result.x1.sin().abs() < 1e-6);
    assert!(result.x2.cos().abs() < 1e-6);
}

#[test]
fn test_newton_minimization_no_constraints() {
    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let q = [2.0, -3.0];
    let q_matrix = vec![vec![1.65539, 2.89376], vec![2.89376, 6.51521]];

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(&unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new("-1.3 * x1 + 0.3 * x2 ^ 2").exp();
    cost = cost.add(&nonlinear).wrap();

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState { x1: -0.1, x2: 0.5 };

    let solver =
        NewtonSolver::new_minimization(&cost, None, &unknown_expr, &registry, None).unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();
    let v1 = DVector::from_vec(result.as_vec());
    let v2 = DVector::from_vec(vec![2.0, -3.0]);
    let t = -1.3 * v1[0] + 0.3 * v1[1].powf(2.0);
    let v3 = DVector::from_vec(vec![-1.3 * f64::exp(t), 2.0 * 0.3 * v1[1] * f64::exp(t)]);
    let m = DMatrix::from_row_slice(
        2,
        2,
        q_matrix
            .into_iter()
            .flatten()
            .collect::<Vec<f64>>()
            .as_slice(),
    );
    let r = m * v1 + v2 + v3;
    assert!(r.norm() < 1e-6);
}

#[test]
fn test_gauss_newton_minimization_no_constraints() {
    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let q = [2.0, -3.0];
    let q_matrix = vec![vec![1.65539, 2.89376], vec![2.89376, 6.51521]];

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(&unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new("-1.3 * x1 + 0.3 * x2 ^ 2").exp();
    cost = cost.add(&nonlinear).wrap();

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState { x1: -0.1, x2: 0.5 };
    let mut options = OptimizerConfig::default();
    options.set_gauss_newton(true);

    let solver =
        NewtonSolver::new_minimization(&cost, None, &unknown_expr, &registry, Some(options))
            .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();
    let v1 = DVector::from_vec(result.as_vec());
    let v2 = DVector::from_vec(vec![2.0, -3.0]);
    let t = -1.3 * v1[0] + 0.3 * v1[1].powf(2.0);
    let v3 = DVector::from_vec(vec![-1.3 * f64::exp(t), 2.0 * 0.3 * v1[1] * f64::exp(t)]);
    let m = DMatrix::from_row_slice(
        2,
        2,
        q_matrix
            .into_iter()
            .flatten()
            .collect::<Vec<f64>>()
            .as_slice(),
    );
    let r = m * v1 + v2 + v3;
    assert!(r.norm() < 1e-6);
}

#[test]
fn test_newton_minimization() {
    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let q = [2.0, -3.0];
    let q_matrix = vec![vec![1.65539, 2.89376], vec![2.89376, 6.51521]];

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(&unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new("-1.3 * x1 + 0.3 * x2 ^ 2").exp();
    cost = cost.add(&nonlinear).wrap();

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState2 {
        x1: -0.1,
        x2: 0.5,
        lambda_0: 0.0,
    };
    let eq_constraints_expr = unknown_expr
        .norm2()
        .unwrap()
        .sub(&ExprScalar::new("0.5"))
        .wrap();

    let solver = NewtonSolver::new_minimization(
        &cost,
        Some(ExprVector::from_vec(vec![eq_constraints_expr])),
        &unknown_expr,
        &registry,
        None,
    )
    .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();
    let v1 = DVector::from_vec(result.as_vec());
    let v2 = DVector::from_vec(vec![2.0, -3.0]);
    let t = -1.3 * v1[0] + 0.3 * v1[1].powf(2.0);
    let v3 = DVector::from_vec(vec![-1.3 * f64::exp(t), 2.0 * 0.3 * v1[1] * f64::exp(t)]);
    let m = DMatrix::from_row_slice(
        2,
        2,
        q_matrix
            .into_iter()
            .flatten()
            .collect::<Vec<f64>>()
            .as_slice(),
    );
    let r2 = (v1[0] * v1[0] + v1[1] * v1[1]).powf(0.5) - 0.5;
    assert!(r2 < 1e-6);
}

#[test]
fn test_gauss_newton_minimization() {
    let unknown_expr = ExprVector::new(&vec!["x1", "x2"]);
    let q = [2.0, -3.0];
    let q_matrix = vec![vec![1.65539, 2.89376], vec![2.89376, 6.51521]];

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(&unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new("-1.3 * x1 + 0.3 * x2 ^ 2").exp();
    cost = cost.add(&nonlinear).wrap();

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = MyState2 {
        x1: -0.1,
        x2: 0.5,
        lambda_0: 0.0,
    };
    let eq_constraints_expr = unknown_expr
        .norm2()
        .unwrap()
        .sub(&ExprScalar::new("0.5"))
        .wrap();
    let mut options = OptimizerConfig::default();
    options.set_gauss_newton(true);

    let solver = NewtonSolver::new_minimization(
        &cost,
        Some(ExprVector::from_vec(vec![eq_constraints_expr])),
        &unknown_expr,
        &registry,
        Some(options),
    )
    .unwrap();
    let result = solver.solve(&initial_guess, &registry).unwrap();
    let v1 = DVector::from_vec(result.as_vec());
    let v2 = DVector::from_vec(vec![2.0, -3.0]);
    let t = -1.3 * v1[0] + 0.3 * v1[1].powf(2.0);
    let v3 = DVector::from_vec(vec![-1.3 * f64::exp(t), 2.0 * 0.3 * v1[1] * f64::exp(t)]);
    let m = DMatrix::from_row_slice(
        2,
        2,
        q_matrix
            .into_iter()
            .flatten()
            .collect::<Vec<f64>>()
            .as_slice(),
    );
    let r2 = (v1[0] * v1[0] + v1[1] * v1[1]).powf(0.5) - 0.5;
    assert!(r2 < 1e-6);
}
