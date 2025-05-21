use control_rs::numeric_services::solver::{NewtonSolverSymbolic, OptimizerConfig};
use control_rs::numeric_services::symbolic::{ExprRegistry, ExprScalar, ExprVector};
use control_rs::plotter;
use std::io::{self, Read};
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

/// c1: norm(x) - 0.5 = 0
fn get_eq_constraints_expr(unknown_expr: &ExprVector) -> ExprVector {
    let constraint1 = unknown_expr
        .norm2()
        .unwrap()
        .sub(&ExprScalar::new("0.5"))
        .wrap();
    ExprVector::from_vec(vec![constraint1])
}

/// c1: -x1^2 - 2 * x2^2 + 0.5 >= 0
fn get_ineq_constraints_expr(unknown_expr: &ExprVector) -> ExprVector {
    let x1_sq = unknown_expr.get(0).unwrap().pow(2.0);
    let x2_sq = unknown_expr.get(1).unwrap().pow(2.0);
    let constraint = x1_sq
        .add(&x2_sq.scalef(2.0))
        .sub(&ExprScalar::from_f64(0.5))
        .wrap()
        .scalef(-1.0)
        .wrap();
    ExprVector::from_vec(vec![constraint])
}

fn main() {
    env_logger::init();
    let unknown_expr = ExprVector::new(&["x1", "x2"]);
    let cost = get_cost_expr(&unknown_expr);

    let registry = Arc::new(ExprRegistry::new());
    let mut initial_guess = vec![-0.1, 0.5];
    let eq_constraints_expr = get_eq_constraints_expr(&unknown_expr);
    let ineq_constraints_expr = get_ineq_constraints_expr(&unknown_expr);
    // set optimization to 1 iterations so that we can draw convergence
    let mut options = OptimizerConfig::default();
    options.set_max_iters(1).unwrap();
    options.set_verbose(true);

    let solver = NewtonSolverSymbolic::new_minimization(
        &cost,
        Some(eq_constraints_expr.clone()),
        //None,
        Some(ineq_constraints_expr.clone()),
        &unknown_expr,
        &registry,
        Some(options),
    )
    .unwrap();

    let mut iter = 0;
    let mut history = Vec::new();

    loop {
        match solver.solve(&initial_guess) {
            Ok((result, _, _, _)) => {
                dbg!(&result);
                history.push(result.clone());
                initial_guess = result;
            }
            Err(e) => {
                eprintln!("Solver error: {e:?}");
                break;
            }
        }

        let filename = "/tmp/plot1.png";
        plotter::plot_minimization(
            &cost,
            &history,
            Some(eq_constraints_expr.clone()),
            Some(ineq_constraints_expr.clone()),
            &unknown_expr,
            filename,
            &registry,
        )
        .unwrap();
        plotter::display(filename).unwrap();
        iter += 1;

        println!("Iteration {iter}. Press Enter to step or q to quit:");
        let mut buf = [0u8; 1];
        io::stdin().read_exact(&mut buf).unwrap();
        if buf[0] == b'q' {
            break;
        }
    }
}
