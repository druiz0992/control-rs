use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::{RK4Symbolic, SymbolicDiscretizer};
use control_rs::physics::models::{CartPole, DoublePendulum, Quadrotor2D};
use control_rs::physics::traits::{Dynamics, SymbolicDynamics};
use control_rs::utils::Labelizable;
use control_rs::utils::evaluable::EvaluableMatrixFn;
use nalgebra::DMatrix;
use once_cell::sync::Lazy;
use proptest::prelude::*;
use std::f64::consts::PI;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;
use std::time::Instant;

#[derive(Default)]
struct TimingStats {
    symbolic_total: Duration,
    numeric_total: Duration,
    iterations: usize,
}

// Double Pendulum
static DOUBLE_PENDULUM_REGISTRY: Lazy<Arc<ExprRegistry>> =
    Lazy::new(|| Arc::new(ExprRegistry::new()));
static DOUBLE_PENDULUM_DF_DX_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &DOUBLE_PENDULUM_REGISTRY;
    let model = DoublePendulum::new(1.0, 1.0, 1.0, 1.0, 0.1, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_x().unwrap()
});
static DOUBLE_PENDULUM_DF_DU_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &DOUBLE_PENDULUM_REGISTRY;
    let model = DoublePendulum::new(1.0, 1.0, 1.0, 1.0, 0.1, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_u().unwrap()
});
static DOUBLE_PENDULUM_TIMINGS: Lazy<Mutex<TimingStats>> =
    Lazy::new(|| Mutex::new(TimingStats::default()));

// Cart Pole
static CARTPOLE_REGISTRY: Lazy<Arc<ExprRegistry>> = Lazy::new(|| Arc::new(ExprRegistry::new()));
static CARTPOLE_DF_DX_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &CARTPOLE_REGISTRY;
    let model = CartPole::new(1.0, 1.0, 1.0, 0.1, 0.1, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_x().unwrap()
});
static CARTPOLE_DF_DU_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &CARTPOLE_REGISTRY;
    let model = CartPole::new(1.0, 1.0, 1.0, 0.1, 0.1, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_u().unwrap()
});
static CARTPOLE_TIMINGS: Lazy<Mutex<TimingStats>> =
    Lazy::new(|| Mutex::new(TimingStats::default()));

// Quadrotor 2D
static QUADROTOR_2D_REGISTRY: Lazy<Arc<ExprRegistry>> = Lazy::new(|| Arc::new(ExprRegistry::new()));
static QUADROTOR_2D_DF_DX_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &QUADROTOR_2D_REGISTRY;
    let model = Quadrotor2D::new(1.0, 1.0, 1.0, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_x().unwrap()
});
static QUADROTOR_2D_DF_DU_FN: Lazy<EvaluableMatrixFn> = Lazy::new(|| {
    let registry = &QUADROTOR_2D_REGISTRY;
    let model = Quadrotor2D::new(1.0, 1.0, 1.0, Some(registry), false);
    let integrator = RK4Symbolic::new(&model, Arc::clone(registry)).unwrap();
    integrator.jacobian_u().unwrap()
});
static QUADROTOR_2D_TIMINGS: Lazy<Mutex<TimingStats>> =
    Lazy::new(|| Mutex::new(TimingStats::default()));

fn insert_vars<M: Dynamics + Labelizable>(
    state_vals: &[f64],
    input_vals: &[f64],
    model_vals: &[f64],
    dt: f64,
    registry: &Arc<ExprRegistry>,
) {
    let model_labels = M::labels();
    let state_labels = M::State::labels();
    let input_labels = M::Input::labels();
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    for (label, &value) in model_labels.iter().zip(model_vals) {
        registry.insert_var(label, value);
    }
    for (label, &value) in input_labels.iter().zip(input_vals) {
        registry.insert_var(label, value);
    }
    for (label, &value) in state_labels.iter().zip(state_vals) {
        registry.insert_var(label, value);
    }
}

fn vectorize_params(
    state_vals: &[f64],
    input_vals: &[f64],
    model_vals: &[f64],
    dt: f64,
) -> Vec<f64> {
    let mut params = Vec::with_capacity(model_vals.len() + state_vals.len() + input_vals.len() + 1);
    params.extend_from_slice(state_vals);
    params.extend_from_slice(input_vals);
    params.extend_from_slice(model_vals);
    params.push(dt);

    params
}

type Func = Box<dyn Fn(&[f64]) -> DMatrix<f64>>;
#[allow(clippy::too_many_arguments)]
fn compare_symbolic_numeric<D: SymbolicDynamics + Labelizable>(
    _model: D,
    model_vals: &[f64],
    state_vals: &[f64],
    input_vals: &[f64],
    dt: f64,
    df_dx_fn: &EvaluableMatrixFn,
    df_du_fn: &EvaluableMatrixFn,
    df_dx: Func,
    df_du: Func,
    timings: &Lazy<Mutex<TimingStats>>,
    registry: &Arc<ExprRegistry>,
) {
    insert_vars::<D>(state_vals, input_vals, model_vals, dt, registry);
    let params = vectorize_params(state_vals, input_vals, model_vals, dt);

    let start = Instant::now();
    let symbolic_df_dx = df_dx_fn.evaluate(&params).unwrap();
    let symbolic_df_du = df_du_fn.evaluate(&params).unwrap();
    let symbolic_duration = start.elapsed();

    let start = Instant::now();
    let numeric_df_dx = df_dx(&params);
    let numeric_df_du = df_du(&params);
    let numeric_duration = start.elapsed();

    {
        let mut stats = timings.lock().unwrap();
        stats.symbolic_total += symbolic_duration;
        stats.numeric_total += numeric_duration;
        stats.iterations += 2;
    }

    assert!((symbolic_df_dx - numeric_df_dx).norm() <= 1e-3);
    assert!((symbolic_df_du - numeric_df_du).norm() <= 1e-3);
}

fn show_stats(timings: &Lazy<Mutex<TimingStats>>) {
    let stats = timings.try_lock().unwrap();
    println!(
        "\n\n=== Timing stats ({} iterations) ===\nSymbolic total: {:?}\nNumeric total: {:?}\nAvg symbolic: {:?}\nAvg numeric: {:?}",
        stats.iterations,
        stats.symbolic_total,
        stats.numeric_total,
        stats.symbolic_total / stats.iterations as u32,
        stats.numeric_total / stats.iterations as u32
    );
}

#[test]
fn test_double_pendulum_random() {
    proptest!(|(
        m1 in 0.5f64..2.0,
        m2 in 0.5f64..2.0,
        l1 in 0.5f64..2.0,
        l2 in 0.5f64..2.0,
        air_resistance_coeff in 0.0f64..0.5,
        theta1 in -PI..PI,
        omega1 in -5.0f64..5.0,
        theta2 in -PI..PI,
        omega2 in -5.0f64..5.0,
        u1 in -10.0f64..10.0,
        u2 in -10.0f64..10.0)|
     {
        let model_vals = [m1, m2, l1, l2, air_resistance_coeff];
        let state_vals = [theta1, omega1, theta2, omega2];
        let input_vals = [u1, u2];
        let dt = 0.01;

        let registry = &DOUBLE_PENDULUM_REGISTRY;
        let df_dx_fn = &DOUBLE_PENDULUM_DF_DX_FN;
        let df_du_fn = &DOUBLE_PENDULUM_DF_DU_FN;
        let timings = &DOUBLE_PENDULUM_TIMINGS;

        let df_dx = Box::new(ffi_codegen::rk4::double_pendulum::rk4_double_pendulum_jacobian_x);
        let df_du = Box::new(ffi_codegen::rk4::double_pendulum::rk4_double_pendulum_jacobian_u);

        let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(registry), false);
        compare_symbolic_numeric(model, &model_vals, &state_vals, &input_vals, dt, df_dx_fn, df_du_fn, df_dx, df_du, timings, registry);
    });
    let timings = &DOUBLE_PENDULUM_TIMINGS;
    show_stats(timings);
}

#[test]
fn test_cartpole_random() {
    proptest!(|(
        pole_mass in 0.1f64..2.0,
        cart_mass in 0.1f64..2.0,
        l in 0.5f64..2.0,
        friction_coeff in 0.0f64..0.5,
        air_resistance_coeff in 0.0f64..0.5,
        pos_x in -3.0f64..3.0,
        v_x in -5.0f64..5.0,
        theta in -PI..PI,
        omega in -5.0f64..5.0,
        u1 in -10.0f64..10.0)|
    {
        let model_vals = [pole_mass, cart_mass, l, friction_coeff, air_resistance_coeff];
        let state_vals = [pos_x, v_x, theta, omega];
        let input_vals = [u1];
        let dt = 0.01;

        let registry = &CARTPOLE_REGISTRY;
        let df_dx_fn = &CARTPOLE_DF_DX_FN;
        let df_du_fn = &CARTPOLE_DF_DU_FN;
        let timings = &CARTPOLE_TIMINGS;
        let df_dx = Box::new(ffi_codegen::rk4::cartpole::rk4_cartpole_jacobian_x);
        let df_du = Box::new(ffi_codegen::rk4::cartpole::rk4_cartpole_jacobian_u);

        let model = CartPole::new(pole_mass, cart_mass,l,friction_coeff, air_resistance_coeff, Some(registry), false);
        compare_symbolic_numeric(model, &model_vals, &state_vals, &input_vals, dt, df_dx_fn, df_du_fn, df_dx, df_du, timings, registry);
    });
    let timings = &CARTPOLE_TIMINGS;
    show_stats(timings);
}

#[test]
fn test_quadrotor_2d_random() {
    proptest!(|(
        m in 0.5f64..2.0,
        j in 0.5f64..2.0,
        l in 0.5f64..2.0,
        pos_x in -3.0f64..3.0,
        pos_y in -5.0f64..5.0,
        theta in -PI..PI,
        v_x in -5.0f64..5.0,
        v_y in -5.0f64..5.0,
        omega in -5.0f64..5.0,
        u1 in -10.0f64..10.0,
        u2 in -10.0f64..10.0)|
     {
        let model_vals = [m,j, l];
        let state_vals = [pos_x, pos_y, theta, v_x, v_y, omega];
        let input_vals = [u1, u2];
        let dt = 0.01;

        let registry = &QUADROTOR_2D_REGISTRY;
        let df_dx_fn = &QUADROTOR_2D_DF_DX_FN;
        let df_du_fn = &QUADROTOR_2D_DF_DU_FN;
        let timings = &QUADROTOR_2D_TIMINGS;
        let df_dx = Box::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_jacobian_x);
        let df_du = Box::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_jacobian_u);

        let model = Quadrotor2D::new(m,j,l,Some(registry), false);
        compare_symbolic_numeric(model, &model_vals, &state_vals, &input_vals, dt, df_dx_fn, df_du_fn, df_dx, df_du, timings, registry);
    });
    let timings = &QUADROTOR_2D_TIMINGS;
    show_stats(timings);
}
