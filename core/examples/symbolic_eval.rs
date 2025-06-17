use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::models::{DoublePendulum, DoublePendulumInput, DoublePendulumState};
use control_rs::physics::traits::SymbolicDynamics;
use control_rs::utils::Labelizable;
use std::fs;
use std::sync::Arc;

fn main() {
    let registry = Arc::new(ExprRegistry::new());

    let pendulum = DoublePendulum::new(1.0, 1.0, 1.0, 1.0, 1.0, Some(&registry));
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
    let dynamics_expr = pendulum.dynamics_symbolic(&state_symbol, &registry);

    let states = DoublePendulumState::labels();
    let actions = DoublePendulumInput::labels();
    let mut params = Vec::with_capacity(states.len() + actions.len());

    params.extend_from_slice(states);
    params.extend_from_slice(actions);

    let slab_df_dx = dynamics_expr
        .jacobian(&state_symbol)
        .unwrap()
        .get_slab()
        .unwrap();

    let slab_df_du = dynamics_expr
        .jacobian(&input_symbol)
        .unwrap()
        .get_slab()
        .unwrap();

    let mut code = slab_df_dx.generate_fn_from_slab("eval_dfdx", &params);
    code += slab_df_du
        .generate_fn_from_slab("eval_dfdu", &params)
        .as_str();

    fs::write("./core/examples/rk4_numeric.rs", code).unwrap();
}

/*

use std::f64::consts::PI;
use std::sync::Arc;

use control_rs::numeric_services::symbolic::{
    ExprRegistry, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::rk4::RK4Numeric;
use control_rs::physics::discretizer::{NumericDiscretizer, RK4Symbolic, SymbolicDiscretizer};
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::traits::{Dynamics, State, SymbolicDynamics};
use control_rs::utils::evaluable::Evaluable;
use control_rs::utils::matrix::vec_to_dmat;
use nalgebra::{DMatrix, DVector};
use std::time::Instant;

fn main() {
    let m1 = 1.0;
    let m2 = 1.0;
    let l1 = 1.0;
    let l2 = 1.0;
    let air_resistance_coeff = 0.0;

    let theta1 = PI / 1.6;
    let omega1 = 0.0;
    let theta2 = PI / 1.8;
    let omega2 = 0.0;
    let dt = 0.01;

    let registry = Arc::new(ExprRegistry::new());

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
    let state_0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
    let jacobian_symbols = state_symbol.extend(&input_symbol);
    let symbolic_f = model.dynamics_symbolic(&state_symbol, &registry);

    let params_state = DVector::from_vec(vec![theta1, omega1, theta2, omega2]);
    let params_input = DVector::from_vec(vec![0.0, 0.0]);
    let mut params = params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());

    let symbolic_eval_f: DVector<f64> =
        SymbolicFunction::new(symbolic_f.to_fn(&registry).unwrap(), &jacobian_symbols)
            .eval(&params)
            .try_into_eval_result()
            .unwrap();

    let symbolic_f_dx = SymbolicFunction::new(
        symbolic_f
            .jacobian(&state_symbol)
            .unwrap()
            .to_fn(&registry)
            .unwrap(),
        &jacobian_symbols,
    )
    .evaluate(&params)
    .unwrap();
    let symbolic_f_du = SymbolicFunction::new(
        symbolic_f
            .jacobian(&input_symbol)
            .unwrap()
            .to_fn(&registry)
            .unwrap(),
        &jacobian_symbols,
    )
    .evaluate(&params)
    .unwrap();

    /////
    let discretizer = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    let fa = discretizer.jacobian_x().unwrap();
    let fb = discretizer.jacobian_u().unwrap();
    let start = Instant::now();
    let a = fa.evaluate(&params).unwrap();
    let b = fb.evaluate(&params).unwrap();
    let duration_symbolic = start.elapsed();

    //let c = discretizer.step(&model, &state_0, None, dt).unwrap();

    let start = Instant::now();

    let df_dx = eval_dfdx(&params);
    let df_du = eval_dfdu(&params);
    let f = model.dynamics(&state_0, None).to_vec();

    //RK4
    let k1 = DVector::from_vec(f.clone());
    let dk1_dx = &df_dx;
    let dk1_du = &df_du;

    let new_params_state = &params_state + dt / 2.0 * &k1;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    let k2 = model
        .dynamics(
            &DoublePendulumState::from_slice(new_params_state.as_slice()),
            None,
        )
        .to_vector();
    let a_k2 = eval_dfdx(&params);
    let b_k2 = eval_dfdu(&params);
    let tmp_dx = &a_k2 * dk1_dx * dt / 2.0;
    let dk2_dx = &tmp_dx + &a_k2;
    let tmp_du = &a_k2 * dk1_du * dt / 2.0;
    let dk2_du = &tmp_du + b_k2;

    let new_params_state = &params_state + dt / 2.0 * &k2;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    let k3 = model
        .dynamics(
            &DoublePendulumState::from_slice(new_params_state.as_slice()),
            None,
        )
        .to_vector();
    let a_k3 = eval_dfdx(&params);
    let b_k3 = eval_dfdu(&params);
    let tmp_dx = &a_k3 * &dk2_dx * dt / 2.0;
    let dk3_dx = &tmp_dx + &a_k3;
    let tmp_du = &a_k3 * &dk2_du * dt / 2.0;
    let dk3_du = &tmp_du + b_k3;

    let new_params_state = &params_state + dt * &k3;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    //let k4 = DVector::from_vec(eval_f(&params));
    let a_k4 = eval_dfdx(&params);
    let b_k4 = eval_dfdu(&params);
    let tmp_dx = &a_k4 * &dk3_dx * dt;
    let dk4_dx = &tmp_dx + &a_k4;
    let tmp_du = &a_k4 * &dk3_du * dt;
    let dk4_du = &tmp_du + b_k4;

    let dx_next_dx =
        dt / 6.0 * (dk1_dx + 2.0 * dk2_dx + 2.0 * dk3_dx + dk4_dx) + DMatrix::identity(4, 4);
    let dx_next_du = dt / 6.0 * (dk1_du + 2.0 * dk2_du + 2.0 * dk3_du + dk4_du);
    let duration_numeric = start.elapsed();

    println!("{}, {:?}", symbolic_f_dx, df_dx);
    println!("{}, {:?}", symbolic_f_du, df_du);
    println!("{}, {:?}", symbolic_eval_f, f);
    println!("{}, {}", dx_next_dx, a);
    println!("{}, {}", dx_next_du, b);

    println!("{:?}, {:?}", duration_symbolic, duration_numeric);

    //
    let discretizer = RK4Numeric::new(&model, Box::new(eval_dfdx), Box::new(eval_dfdu)).unwrap();
    let (j1, j2) = discretizer.jacobians(&model, &state_0, None, dt);
    println!("{}, {}", j1, j2);
}

*/
