use control_rs::numeric_services::solver::{
    LineSeachConfig, NewtonSolverSymbolic, OptimizerConfig,
};
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::numeric_services::symbolic::fasteval::utils::*;
use control_rs::numeric_services::symbolic::{
    ExprScalar, ExprVector, SymbolicEvalResult, SymbolicExpr, SymbolicFunction,
};
use control_rs::solver::Minimizer;
use control_rs::solver::osqp::builder::OSQPBuilder;
use control_rs::solver::qp::QPBuilder;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

const Q_LINEAR: [f64; 2] = [2.0, -3.0];
const Q_QUADRATIC: [[f64; 2]; 2] = [[1.65539, 2.89376], [2.89376, 6.51521]];
const Q_NONLINEAL: &str = "-1.3 * x1 + 0.3 * x2 ^ 2";

/// Cost = 0.5 * x'Qx + q'x + exp(-1.3 * x[0] + 0.3 * x[1]^2)
fn get_cost_expr(unknown_expr: &ExprVector) -> ExprScalar {
    let q = Q_LINEAR;
    let q_matrix = Q_QUADRATIC.map(|row| row.to_vec()).to_vec();

    let mut cost = unknown_expr
        .scalef(0.5)
        .vecmul_matf(&q_matrix)
        .wrap()
        .dot(unknown_expr)
        .unwrap();
    cost = cost.add(&unknown_expr.dotf(&q).unwrap());
    let nonlinear = ExprScalar::new(Q_NONLINEAL).exp();
    cost.add(&nonlinear).wrap()
}

fn grad_cost_norm(x: &[f64]) -> f64 {
    let x = DVector::from_vec(x.to_vec());
    let q = DVector::from_vec(Q_LINEAR.to_vec());
    let q_matrix = DMatrix::from_vec(2, 2, Q_QUADRATIC.concat().to_vec());

    let linear_grad = &q_matrix * &x + q;

    let t = -1.3 * x[0] + 0.3 * x[1].powi(2);
    let exp_term = f64::exp(t);
    let nonlinear_grad = DVector::from_vec(vec![-1.3 * exp_term, 2.0 * 0.3 * x[1] * exp_term]);

    (linear_grad + nonlinear_grad).norm()
}

fn eval_eq_constraint(x: &[f64]) -> f64 {
    (x[0] * x[0] + x[1] * x[1]).powf(0.5) - 0.5
}

fn eval_ineq_constraint(x: &[f64]) -> f64 {
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

fn get_ineq_constraints_expr(unknown_expr: &ExprVector) -> ExprVector {
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
    let solver =
        NewtonSolverSymbolic::new_root_solver(&expr, &unknown_expr, &registry, None).unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    assert!(result[0].sin().abs() < 1e-6);
    assert!(result[1].cos().abs() < 1e-6);
}

#[test]
fn test_newton_root_finding_norm1_line_search() {
    let expr = ExprVector::new(&["sin(x1)", "cos(x2)"]);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-1.742410372590328, 1.4020334125022704];

    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let ls_opts = LineSeachConfig::default().set_merit(Some(expr.norm1().unwrap()));
    let options = OptimizerConfig::default().set_line_search_opts(ls_opts);

    let solver =
        NewtonSolverSymbolic::new_root_solver(&expr, &unknown_expr, &registry, Some(options))
            .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

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
        NewtonSolverSymbolic::new_minimization(&cost, None, None, &unknown_expr, &registry, None)
            .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let grad_cost_norm = grad_cost_norm(&result);
    assert!(grad_cost_norm < 1e-6);
}

#[test]
fn test_gauss_newton_minimization_no_constraints() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let options = OptimizerConfig::default().set_gauss_newton(true);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        None,
        None,
        &unknown_expr,
        &registry,
        Some(options),
    )
    .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let grad_cost_norm = grad_cost_norm(&result);
    assert!(grad_cost_norm < 1e-6);
}

#[test]
fn test_newton_minimization() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        Some(&eq_constraints_expr),
        None,
        &unknown_expr,
        &registry,
        None,
    )
    .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let constraint_eval = eval_eq_constraint(&result);

    assert!(constraint_eval < 1e-6);
}

#[test]
fn test_gauss_newton_minimization() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);

    let options = OptimizerConfig::default().set_gauss_newton(true);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        Some(&eq_constraints_expr),
        None,
        &unknown_expr,
        &registry,
        Some(options),
    )
    .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let constraint_eval = eval_eq_constraint(&result);

    assert!(constraint_eval < 1e-6);
}

#[test]
fn test_newton_minimization_ineq_eq_constraint() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);
    let ineq_constraints_expr = get_ineq_constraints_expr(&unknown_expr);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        Some(&eq_constraints_expr),
        Some(&ineq_constraints_expr),
        &unknown_expr,
        &registry,
        None,
    )
    .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let eq_constraint_eval = eval_eq_constraint(&result);
    let ineq_constraint_eval = eval_ineq_constraint(&result);

    assert!(eq_constraint_eval < 1e-3);
    assert!(ineq_constraint_eval >= 0.0);
}

#[test]
fn test_newton_minimization_ineq_constraint() {
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let initial_guess = vec![-0.1, 0.5];
    let ineq_constraints_expr = get_ineq_constraints_expr(&unknown_expr);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        None,
        Some(&ineq_constraints_expr),
        &unknown_expr,
        &registry,
        None,
    )
    .unwrap();
    let (result, _, _, _) = solver.solve(&initial_guess).unwrap();

    let ineq_constraint_eval = eval_ineq_constraint(&result);

    assert!(ineq_constraint_eval >= 0.0);
}

#[test]
fn test_symbolic_qp() {
    let registry = Arc::new(ExprRegistry::new());
    let big_q_data = vec![
        1.0, 0.3, 0.0, 0.0, // row 1
        0.3, 1.0, 0.0, 0.0, // row 2
        0.0, 0.0, 2.0, 0.0, // row 3
        0.0, 0.0, 0.0, 4.0, // row 4
    ];

    let big_a_data = vec![0.0, 0.0, 1.0, 1.0, -1.0, 2.3, 1.0, -2.0];

    let identity = DMatrix::<f64>::identity(4, 4);
    let neg_identity = -&identity;

    let primal_max = [1.0, 1.0, 1.0, 1.0];
    let primal_min = [-0.5, -0.5, -1.0, -1.0];
    let neg_primal_max: Vec<f64> = primal_max.iter().map(|x| -x).collect();
    let mut h_data = neg_primal_max;
    h_data.extend(primal_min);

    // objective: 0.5 * X' * Q * X + q * X
    let big_q = DMatrix::from_row_slice(4, 4, &big_q_data);
    let q = DVector::from_vec(vec![-2.0, 3.4, 2.0, 4.0]);
    // eq constraints: A * X - b
    let big_a = DMatrix::from_row_slice(2, 4, &big_a_data);
    let b = DVector::from_vec(vec![1.0, 3.0]);
    // ineq constraints: G*x - h
    let big_g = DMatrix::from_rows(
        &neg_identity
            .row_iter()
            .chain(identity.row_iter())
            .collect::<Vec<_>>(),
    );
    let h = DVector::from_vec(h_data);

    // convert to expr
    let big_q_expr = from_dmatrix(big_q.clone());
    let q_expr = from_dvector(q.clone());

    let big_a_expr = from_dmatrix(big_a.clone());
    let b_expr = from_dvector(b.clone());

    let big_g_expr = from_dmatrix(big_g.clone());
    let h_expr = from_dvector(h.clone());

    //
    let (x, y, w, z) = (1.0, 2.0, 3.0, 4.0);
    let unknown_vector = DVector::from_vec(vec![x, y, w, z]);
    let unknown_vector_expr = ExprVector::new(&["x", "y", "w", "z"]);

    // objective
    let objective = 0.5 * unknown_vector.transpose() * &big_q * &unknown_vector
        + q.transpose() * &unknown_vector;
    let objective_expr = unknown_vector_expr
        .vecmul_mat(&big_q_expr)
        .wrap()
        .dot(&unknown_vector_expr)
        .unwrap()
        .wrap()
        .scalef(0.5)
        .wrap()
        .add(&q_expr.dot(&unknown_vector_expr).unwrap().wrap())
        .wrap();
    let objective_fn = SymbolicFunction::new(
        objective_expr.to_fn(&registry).unwrap(),
        &unknown_vector_expr,
    );

    // eq
    let eq_constraints = big_a * &unknown_vector - b;
    let eq_constraints_expr = big_a_expr
        .matmul_vec(&unknown_vector_expr)
        .wrap()
        .sub(&b_expr)
        .wrap();
    let eq_constraints_fn = SymbolicFunction::new(
        eq_constraints_expr.to_fn(&registry).unwrap(),
        &unknown_vector_expr,
    );

    // ineq
    let ineq_constraints = big_g * &unknown_vector - h;
    let ineq_constraints_expr = big_g_expr
        .matmul_vec(&unknown_vector_expr)
        .wrap()
        .sub(&h_expr)
        .wrap();
    let ineq_constraints_fn = SymbolicFunction::new(
        ineq_constraints_expr.to_fn(&registry).unwrap(),
        &unknown_vector_expr,
    );

    // check expressiosn are equal
    if let Ok(SymbolicEvalResult::Scalar(result)) = objective_fn.eval(unknown_vector.as_slice()) {
        assert!(
            (objective[(0, 0)] - result).abs() < 1e-3,
            "Objective function doesnt match"
        );
    } else {
        panic!("Unexpected result for objective function");
    }
    if let Ok(SymbolicEvalResult::Vector(result)) =
        eq_constraints_fn.eval(unknown_vector.as_slice())
    {
        assert!(
            (eq_constraints - result).sum().abs() < 1e-3,
            "Equality constraints don't match"
        );
    } else {
        panic!("Unexpected result for equality constraints");
    }

    if let Ok(SymbolicEvalResult::Vector(result)) =
        ineq_constraints_fn.eval(unknown_vector.as_slice())
    {
        assert!(
            (ineq_constraints - result).sum().abs() < 1e-3,
            "Inequality constraints don't match"
        );
    } else {
        panic!("Unexpected result for inequality constraints");
    }

    let solver_options = OptimizerConfig::default().set_verbose(true);

    let solver = NewtonSolverSymbolic::new_minimization(
        &objective_expr,
        Some(&eq_constraints_expr),
        Some(&ineq_constraints_expr),
        &unknown_vector_expr,
        &registry,
        Some(solver_options),
    )
    .unwrap();
    let (_, status, _, _) = solver.solve(&[0.0, 0.0, 0.0, 0.0]).unwrap();

    let tol = 1e-5;
    assert!(status.stationarity < tol);
    assert!(status.max_primal_feasibility_c.unwrap() < tol);
    assert!(status.min_primal_feasibility_h.unwrap() > -tol);
    assert!(status.dual_feasibility.unwrap() > -tol);
    assert!(status.complementary_slackness.unwrap() < tol);
}

#[test]
fn test_qp() {
    let big_q_data = vec![
        1.0, 0.3, 0.0, 0.0, // row 1
        0.3, 1.0, 0.0, 0.0, // row 2
        0.0, 0.0, 2.0, 0.0, // row 3
        0.0, 0.0, 0.0, 4.0, // row 4
    ];

    let big_a_data = vec![0.0, 0.0, 1.0, 1.0, -1.0, 2.3, 1.0, -2.0];

    let identity = DMatrix::<f64>::identity(4, 4);
    let neg_identity = -&identity;

    let primal_max = [1.0, 1.0, 1.0, 1.0];
    let primal_min = [-0.5, -0.5, -1.0, -1.0];
    let neg_primal_max: Vec<f64> = primal_max.iter().map(|x| -x).collect();
    let mut h_data = neg_primal_max;
    h_data.extend(primal_min);

    // objective: 0.5 * X' * Q * X + q * X
    let big_q = DMatrix::from_row_slice(4, 4, &big_q_data);
    let q = DVector::from_vec(vec![-2.0, 3.4, 2.0, 4.0]);
    // eq constraints: A * X - b
    let big_a = DMatrix::from_row_slice(2, 4, &big_a_data);
    let b = DVector::from_vec(vec![1.0, 3.0]);
    // ineq constraints: G*x - h
    let big_g = DMatrix::from_rows(
        &neg_identity
            .row_iter()
            .chain(identity.row_iter())
            .collect::<Vec<_>>(),
    );
    let h = DVector::from_vec(h_data);

    let qp_builder = QPBuilder::new();
    let solver = qp_builder
        .q_mat(big_q)
        .q_vec(q)
        .a_mat(big_a)
        .b_vec(b)
        .g_mat(big_g)
        .h_vec(h)
        .build()
        .unwrap();

    let (_, status, _, _) = solver.solve_qp(&[0.0, 0.0, 0.0, 0.0]).unwrap();

    let tol = 1e-5;
    dbg!(&status);
    assert!(status.stationarity < tol);
    assert!(status.max_primal_feasibility_c.unwrap() < tol);
    assert!(status.min_primal_feasibility_h.unwrap() > -tol);
    assert!(status.dual_feasibility.unwrap() > -tol);
    assert!(status.complementary_slackness.unwrap() < tol);
}

#[test]
fn test_osqp_handle() {
    let big_q_data = vec![
        1.0, 0.3, 0.0, 0.0, // row 1
        0.3, 1.0, 0.0, 0.0, // row 2
        0.0, 0.0, 2.0, 0.0, // row 3
        0.0, 0.0, 0.0, 4.0, // row 4
    ];

    let big_a_data = vec![0.0, 0.0, 1.0, 1.0, -1.0, 2.3, 1.0, -2.0];

    let identity = DMatrix::<f64>::identity(4, 4);
    let neg_identity = -&identity;

    let primal_max = [1.0, 1.0, 1.0, 1.0];
    let primal_min = [-0.5, -0.5, -1.0, -1.0];
    let neg_primal_max: Vec<f64> = primal_max.iter().map(|x| -x).collect();
    let mut h_data = neg_primal_max;
    h_data.extend(primal_min);

    // objective: 0.5 * X' * Q * X + q * X
    let big_q = DMatrix::from_row_slice(4, 4, &big_q_data);
    let q = DVector::from_vec(vec![-2.0, 3.4, 2.0, 4.0]);
    // eq constraints: A * X - b
    let big_a = DMatrix::from_row_slice(2, 4, &big_a_data);
    let b = DVector::from_vec(vec![1.0, 3.0]);
    // ineq constraints: G*x - h
    let big_g = DMatrix::from_rows(
        &neg_identity
            .row_iter()
            .chain(identity.row_iter())
            .collect::<Vec<_>>(),
    );
    let h = DVector::from_vec(h_data);

    let qp_builder = OSQPBuilder::new();
    let (solver, _) = qp_builder
        .q_mat(big_q)
        .q_vec(q)
        .a_mat(big_a)
        .b_vec(b)
        .g_mat(big_g)
        .h_vec(h)
        .build()
        .unwrap();

    let (_, status, _, _) = solver.minimize(&[]).unwrap();
    let tol = 1e-5;
    assert!(status.min_primal_feasibility_h.unwrap() > -tol);
    assert!(status.dual_feasibility.unwrap() > -tol);
}

#[test]
fn test_osqp_handle_no_eq() {
    let big_q_data = vec![
        1.0, 0.3, 0.0, 0.0, // row 1
        0.3, 1.0, 0.0, 0.0, // row 2
        0.0, 0.0, 2.0, 0.0, // row 3
        0.0, 0.0, 0.0, 4.0, // row 4
    ];

    let identity = DMatrix::<f64>::identity(4, 4);
    let neg_identity = -&identity;

    let primal_max = [1.0, 1.0, 1.0, 1.0];
    let primal_min = [-0.5, -0.5, -1.0, -1.0];
    let neg_primal_max: Vec<f64> = primal_max.iter().map(|x| -x).collect();
    let mut h_data = neg_primal_max;
    h_data.extend(primal_min);

    // objective: 0.5 * X' * Q * X + q * X
    let big_q = DMatrix::from_row_slice(4, 4, &big_q_data);
    let q = DVector::from_vec(vec![-2.0, 3.4, 2.0, 4.0]);
    // ineq constraints: G*x - h
    let big_g = DMatrix::from_rows(
        &neg_identity
            .row_iter()
            .chain(identity.row_iter())
            .collect::<Vec<_>>(),
    );
    let h = DVector::from_vec(h_data);

    let qp_builder = OSQPBuilder::new();
    let (solver, _) = qp_builder
        .q_mat(big_q)
        .q_vec(q)
        .g_mat(big_g)
        .h_vec(h)
        .build()
        .unwrap();

    let (_, status, _, _) = solver.minimize(&[]).unwrap();
    let tol = 1e-5;
    assert!(status.min_primal_feasibility_h.unwrap() > -tol);
    assert!(status.dual_feasibility.unwrap() > -tol);
}

#[test]
fn test_osqp_handle_no_ineq() {
    let big_q_data = vec![
        1.0, 0.3, 0.0, 0.0, // row 1
        0.3, 1.0, 0.0, 0.0, // row 2
        0.0, 0.0, 2.0, 0.0, // row 3
        0.0, 0.0, 0.0, 4.0, // row 4
    ];

    let big_a_data = vec![0.0, 0.0, 1.0, 1.0, -1.0, 2.3, 1.0, -2.0];

    // objective: 0.5 * X' * Q * X + q * X
    let big_q = DMatrix::from_row_slice(4, 4, &big_q_data);
    let q = DVector::from_vec(vec![-2.0, 3.4, 2.0, 4.0]);
    // eq constraints: A * X - b
    let big_a = DMatrix::from_row_slice(2, 4, &big_a_data);
    let b = DVector::from_vec(vec![1.0, 3.0]);

    let qp_builder = OSQPBuilder::new();
    let (solver, _) = qp_builder
        .q_mat(big_q)
        .q_vec(q)
        .a_mat(big_a)
        .b_vec(b)
        .build()
        .unwrap();

    let (_, status, _, _) = solver.minimize(&[]).unwrap();
    let tol = 1e-5;
    assert!(status.min_primal_feasibility_h.unwrap() > -tol);
    assert!(status.dual_feasibility.unwrap() > -tol);
}
