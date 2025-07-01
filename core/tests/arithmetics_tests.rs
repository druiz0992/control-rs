use control_rs::symbolic_services::symbolic::fasteval::{
    ExprMatrix, ExprRegistry, ExprScalar, ExprVector,
};
use control_rs::symbolic_services::symbolic::{SymbolicEvalResult, SymbolicExpr};
use nalgebra::{DMatrix, DVector};
use proptest::prelude::*;
use rand::Rng;
use std::sync::Arc;

fn random_dmatrix(rows: usize, cols: usize) -> DMatrix<f64> {
    let mut rng = rand::thread_rng();
    let data: Vec<f64> = (0..rows * cols).map(|_| rng.gen_range(-1.0..1.0)).collect();
    DMatrix::from_vec(rows, cols, data)
}
fn random_dvector(rows: usize) -> DVector<f64> {
    let mut rng = rand::thread_rng();
    let data: Vec<f64> = (0..rows).map(|_| rng.gen_range(-1.0..1.0)).collect();
    DVector::from_vec(data)
}

fn from_dvector(vector: DVector<f64>, prefix: &str, registry: &Arc<ExprRegistry>) -> ExprVector {
    let expr: Vec<_> = vector
        .iter()
        .enumerate()
        .map(|(idx, &val)| {
            let scalar = format!("{}_{}", prefix, idx);
            registry.insert_var(&scalar, val);
            ExprScalar::from_f64(val)
        })
        .collect();
    ExprVector::from_vec(expr)
}
fn from_dmatrix(matrix: DMatrix<f64>, prefix: &str, registry: &Arc<ExprRegistry>) -> ExprMatrix {
    let rows = matrix.nrows();
    let cols = matrix.ncols();
    let mut expr_matrix = Vec::new();

    for i in 0..rows {
        let mut row = Vec::new();
        for j in 0..cols {
            let scalar = format!("{}_{}_{}", prefix, i, j);
            registry.insert_var(&scalar, matrix[(i, j)]);
            row.push(ExprScalar::new(scalar));
        }
        expr_matrix.push(row);
    }

    ExprMatrix::from_vec(&expr_matrix)
}

// J * (q  + t * v), where J is a MxN matrix, t is a scalar and q,v are N vectors
proptest! {
    #[test]
    fn test_constraint_arithmetic(
        n_cols in 1..10,
        n_rows in 1..10,
        t in -1.0..1.0,
    ) {
        let registry = Arc::new(ExprRegistry::new());

        let j = random_dmatrix(n_rows as usize, n_cols as usize);
        let q = random_dvector(n_cols as usize);
        let v = random_dvector(n_cols as usize);

        let j_expr = from_dmatrix(j.clone(), "j", &registry);
        let q_expr = from_dvector(q.clone(), "q", &registry);
        let v_expr = from_dvector(v.clone(), "v", &registry);
        let t_expr = ExprScalar::new("t");

        registry.insert_var("t", t);

        let target: DVector<f64> = j * (q + t * v);
        let expr = j_expr.matmul_vec(&q_expr.add(&v_expr.scale(&t_expr)).wrap());

        let obtained = expr.to_fn(&registry).unwrap();

        if let Ok(SymbolicEvalResult::Vector(result)) = obtained(None) {
            let result: DVector<f64> = result;
            assert!((target - result).abs().sum() < 1e-3);
        } else {
            panic!("Unexpected result");
        }
  }
}

// J * (q  + t * v), where t is a scalar and q,v,j are N vectors
proptest! {
    #[test]
    fn test_constraint2_arithmetic(
        n_rows in 1..10,
        t in -1.0..1.0,
    ) {
        let registry = Arc::new(ExprRegistry::new());

        let j = random_dvector(n_rows as usize);
        let q = random_dvector(n_rows as usize);
        let v = random_dvector(n_rows as usize);

        let j_expr = from_dvector(j.clone(), "j", &registry);
        let q_expr = from_dvector(q.clone(), "q", &registry);
        let v_expr = from_dvector(v.clone(), "v", &registry);
        let t_expr = ExprScalar::new("t");

        registry.insert_var("t", t);

        let target = j.dot(&(q + t * v));
        let expr = j_expr.dot(&q_expr.add(&v_expr.scale(&t_expr)).wrap()).unwrap();

        let obtained = expr.to_fn(&registry).unwrap();

        if let Ok(SymbolicEvalResult::Scalar(result)) = obtained(None) {
            assert!((target - result).abs() < 1e-3);
        } else {
            panic!("Unexpected result");
        }
  }
}

// 1/2 * v' * M * v + [M (t * g  - x)]' * v, where M is a NxN matrix, g, v and x are a 1xN vector, t is a scalar,
proptest! {
    #[test]
    fn test_objective_arithmetic(
        n_rows in 1..10,
        t in -1.0..1.0,
    ) {
        let registry = Arc::new(ExprRegistry::new());

        let m = random_dmatrix(n_rows as usize, n_rows as usize);
        let x = random_dvector(n_rows as usize);
        let v = random_dvector(n_rows as usize);
        let g = random_dvector(n_rows as usize);

        let m_expr = from_dmatrix(m.clone(), "m", &registry);
        let x_expr = from_dvector(x.clone(), "x", &registry);
        let v_expr = from_dvector(v.clone(), "v", &registry);
        let g_expr = from_dvector(g.clone(), "g", &registry);
        let t_expr = ExprScalar::new("t");

        registry.insert_var("t", t);

        let target = 0.5 * v.transpose() * &m * &v + (m * (t * g - x)).transpose() * v;
        let expr1 = v_expr.vecmul_mat(&m_expr).wrap().dot(&v_expr).unwrap().wrap().scalef(0.5).wrap();
        let expr2 = m_expr.matmul_vec(&g_expr.scale(&t_expr).sub(&x_expr).wrap()).wrap().dot(&v_expr).unwrap().wrap();
        let expr = expr1.add(&expr2);

        let obtained = expr.to_fn(&registry).unwrap();

        if let Ok(SymbolicEvalResult::Scalar(result)) = obtained(None) {
            assert!((target[(0,0)] - result).abs() < 1e-3);
        } else {
            panic!("Unexpected result");
        }
  }
}
