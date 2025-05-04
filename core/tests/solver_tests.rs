use control_rs::numeric_services::symbolic::ExprVector;
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::physics::traits::State;
use control_rs::solver::RootSolver;
use control_rs::solver::models::{LineSeachConfig, OptimizerConfig};
use control_rs::solver::newton::NewtonSolver;
use macros::StateOps;
use std::sync::Arc;

#[derive(StateOps, Debug)]
struct MyState {
    x1: f64,
    x2: f64,
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
