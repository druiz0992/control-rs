use control_rs::numeric_services::symbolic::{SymbolicExpr, SymbolicFunction};
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::rk4_numeric::RK4Numeric;
use control_rs::physics::discretizer::{NumericFunction, RK4Symbolic, SymbolicDiscretizer};
use control_rs::physics::models::double_pendulum::numeric_dynamics::{eval_dfdu, eval_dfdx};
use control_rs::physics::traits::SymbolicDynamics;
use control_rs::utils::Labelizable;
use control_rs::utils::evaluable::Evaluable;
use control_rs::{numeric_services::symbolic::ExprRegistry, physics::models::DoublePendulum};
use nalgebra::DVector;
use once_cell::sync::Lazy;
use proptest::proptest;
use std::{f64::consts::PI, sync::Arc};

const DT: f64 = 0.001;

fn update_registry(registry: &Arc<ExprRegistry>, labels: &[&str], vals: &[f64]) {
    for (label, val) in labels.into_iter().zip(vals.into_iter()) {
        registry.insert_var(label, *val);
    }
}

fn create_symbolic_double_pendulum_jacobians(
    model: &DoublePendulum,
    registry: &Arc<ExprRegistry>,
) -> (SymbolicFunction, SymbolicFunction) {
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, DT);
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
    let jacobian_symbols = state_symbol.extend(&input_symbol);

    let symbolic_f = model.dynamics_symbolic(&state_symbol, registry);

    let dfdx_expr = symbolic_f
        .jacobian(&state_symbol)
        .unwrap()
        .to_fn(registry)
        .unwrap();

    let dfdu_expr = symbolic_f
        .jacobian(&input_symbol)
        .unwrap()
        .to_fn(registry)
        .unwrap();

    let dfdx_symbolic = SymbolicFunction::new(dfdx_expr, &jacobian_symbols);
    let dfdu_symbolic = SymbolicFunction::new(dfdu_expr, &jacobian_symbols);

    (dfdx_symbolic, dfdu_symbolic)
}

static DOUBLE_PENDULUM_REGISTRY: Lazy<Arc<ExprRegistry>> = Lazy::new(|| {
    let registry = Arc::new(ExprRegistry::new());
    let _ = DoublePendulum::new(1.0, 1.0, 1.0, 1.0, 0.0, Some(&registry), false);
    registry
});

static DOUBLE_PENDULUM_SYMBOLIC_MODEL: Lazy<DoublePendulum> = Lazy::new(|| {
    // Use the same params as above — they don't matter for structure
    DoublePendulum::new(
        1.0,
        1.0,
        1.0,
        1.0,
        0.0,
        Some(&DOUBLE_PENDULUM_REGISTRY),
        false,
    )
});

static DOUBLE_PENDULUM_JACOBIANS: Lazy<(SymbolicFunction, SymbolicFunction)> = Lazy::new(|| {
    create_symbolic_double_pendulum_jacobians(
        &DOUBLE_PENDULUM_SYMBOLIC_MODEL,
        &DOUBLE_PENDULUM_REGISTRY,
    )
});

static DOUBLE_PENDULUM_RK4_JACOBIANS: Lazy<(SymbolicFunction, SymbolicFunction)> =
    Lazy::new(|| {
        let rk4 = RK4Symbolic::new(
            &*DOUBLE_PENDULUM_SYMBOLIC_MODEL,
            Arc::clone(&DOUBLE_PENDULUM_REGISTRY),
        )
        .unwrap();

        let djdx = rk4.jacobian_x().unwrap();
        let djdu = rk4.jacobian_u().unwrap();

        (djdx, djdu)
    });

proptest! {
#[test]
fn double_pendulum_test(
            theta1 in 0.0..(2.0 * PI),
            theta2 in 0.0..(2.0 * PI),
            omega1 in -5.0..5.0,
            omega2 in -5.0..5.0,
            u1 in -5.0..5.0,
            u2 in -5.0..5.0,
            m1 in 0.1f64..10.0,
            m2 in 0.1f64..10.0,
            l1 in 0.1f64..5.0,
            l2 in 0.1f64..5.0,
            air_resistance_coeff in 0.0f64..5.0
            ) {
    let dt = DT;
    let tol = 1e-1;
    let registry = &*DOUBLE_PENDULUM_REGISTRY;
    let (dfdx_symbolic_fn, dfdu_symbolic_fn) = &*DOUBLE_PENDULUM_JACOBIANS;
    let params_vals = vec![m1, m2, l1, l2, air_resistance_coeff];

    let model_numeric = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, None, false);

    let params_state = DVector::from_vec(vec![theta1, omega1, theta2, omega2]);
    let params_input = DVector::from_vec(vec![u1, u2]);
    let params_params = DVector::from_column_slice(&params_vals);

    let mut params = params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    params.extend(params_params.as_slice());

    update_registry(registry, DoublePendulum::labels(), &params_vals);

    let dfdx_symbolic = dfdx_symbolic_fn.evaluate(&params).unwrap();
    let dfdu_symbolic = dfdu_symbolic_fn.evaluate(&params).unwrap();

    let dfdx_numeric = eval_dfdx(&params);
    let dfdu_numeric = eval_dfdu(&params);

    // Check dynamic function jacobians
    assert!((&dfdx_numeric - &dfdx_symbolic).norm() < tol, "dfdx mismatch");
    assert!((&dfdu_numeric -  &dfdu_symbolic).norm() < tol, "dfdu mismatch");

    // Closures for continuous dynamics jacobians
    let dfdx = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdx(vals)));
    let dfdu = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdu(vals)));

    // Create numeric RK4 discretizer
    let rk4_numeric = RK4Numeric::new(Arc::new(model_numeric.clone()), dfdx, dfdu, dt).unwrap();
    let djdx_numeric = rk4_numeric.jacobian_x(&params);
    let djdu_numeric = rk4_numeric.jacobian_u(&params);

    let (djdx_symbolic_fn, djdu_symbolic_fn) = &*DOUBLE_PENDULUM_RK4_JACOBIANS;
    let djdx_symbolic = djdx_symbolic_fn.evaluate(&params).unwrap();
    let djdu_symbolic = djdu_symbolic_fn.evaluate(&params).unwrap();

    assert!((djdx_numeric - djdx_symbolic).norm() <= tol);
    assert!((djdu_numeric - djdu_symbolic).norm() <= tol);
}
}
