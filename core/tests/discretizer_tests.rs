use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::{
    BackwardEuler, ForwardEuler, HermiteSimpson, ImplicitMidpoint, MidPoint, RK4, RK4Symbolic, ZOH,
};
use control_rs::physics::models::linear_time_invariant::input::LtiInput;
use control_rs::physics::models::linear_time_invariant::model::LtiModel;
use control_rs::physics::models::linear_time_invariant::state::LtiState;
use control_rs::physics::models::{BouncingBall, BouncingBallState};
use control_rs::physics::traits::{Discretizer, State};
use control_rs::utils::Labelizable;
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

fn init_bouncing_ball() -> (BouncingBall, BouncingBallState, Arc<ExprRegistry>) {
    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 1.0;
    let v_x = 1.0;
    let v_y = 4.5;

    let registry = Arc::new(ExprRegistry::new());
    let state = BouncingBallState::new(pos_x, pos_y, v_x, v_y);

    let model = BouncingBall::new(m, friction_coeff, Some(&registry), true);

    (model, state, registry)
}

#[test]
fn test_explicit_integrators_ko_with_constrained_dynamics() {
    let (model, _, registry) = init_bouncing_ball();

    let rk4 = RK4::new(&model);
    let fe = ForwardEuler::new(&model);
    let mp = MidPoint::new(&model);
    let rk4_symbolic = RK4Symbolic::new(&model, registry);

    assert!(rk4.is_err());
    assert!(rk4_symbolic.is_err());
    assert!(fe.is_err());
    assert!(mp.is_err());
}

#[test]
fn test_hermite_simpson_ko_with_constrained_dynamics() {
    let (model, _, registry) = init_bouncing_ball();

    let hs = HermiteSimpson::new(&model, registry, None);

    assert!(hs.is_err());
}

#[test]
fn test_implicit_ok_with_constrained_dynamics() {
    let (model, _, registry) = init_bouncing_ball();

    let be = BackwardEuler::new(&model, registry.clone(), None);
    let imp = ImplicitMidpoint::new(&model, registry, None);

    assert!(be.is_ok());
    assert!(imp.is_ok());
}

#[test]
fn test_backward_euler_constrained_dynamics() {
    let (model, mut result, registry) = init_bouncing_ball();

    let dt = 0.01;

    let integrator = BackwardEuler::new(&model, registry, None).unwrap();

    for _ in 0..2000 {
        result = integrator.step(&model, &result, None, dt).unwrap();
        let [pos_y] = result.extract(&["pos_y"]);

        // check constain is met
        assert!(pos_y >= 0.0);
    }
}

#[test]
fn test_implicit_midpoint_constrained_dynamics() {
    let (model, mut result, registry) = init_bouncing_ball();

    let dt = 0.01;
    let tol = 1e-1;

    let integrator = ImplicitMidpoint::new(&model, registry, None).unwrap();

    for _ in 0..2000 {
        result = integrator.step(&model, &result, None, dt).unwrap();
        let [pos_y] = result.extract(&["pos_y"]);

        // check constain is met
        assert!(pos_y >= -tol);
    }
}

#[test]
fn test_linear_models() {
    let state_matrix = DMatrix::from_vec(3, 3, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0]);
    let control_matrix = DMatrix::from_vec(3, 2, vec![1.0, 2.0, 0.0, 1.0, 2.0, 1.0]);
    let model = LtiModel::<3, 0, 2>::new(state_matrix.clone(), control_matrix.clone()).unwrap();

    let tol = 1e-5;
    let dt = 0.01;

    let state0 = LtiState::<3, 0>::new([3.0, 2.0, 1.0]);
    let input0 = LtiInput::<2, 0>::new([1.0, 1.0]);

    let dv_state0 = DVector::from_vec(state0.to_vec());
    let dv_input0 = DVector::from_vec(input0.to_vec());

    let fe = ForwardEuler::new(&model).unwrap();
    let md = MidPoint::new(&model).unwrap();
    let rk4 = RK4::new(&model).unwrap();

    let fe_result = fe.step(&model, &state0, Some(&input0), dt).unwrap();
    // r = (I + Ah)x + Bhu
    let fe_expected = &control_matrix * dt * dv_input0.clone()
        + (DMatrix::identity(3, 3) + &state_matrix * dt) * dv_state0.clone();
    let error: f64 = (LtiState::<3, 0>::from_vec(fe_result.to_vec())
        - LtiState::<3, 0>::from_vec(fe_expected.as_slice().to_vec()))
    .to_vec()
    .iter()
    .map(|x| x * x)
    .sum();
    assert!(error < tol);

    let md_result = md.step(&model, &state0, Some(&input0), dt).unwrap();
    // r = (I + Ah + h^2/2 * A^2)x + (Bhu + h^2/2 * A*B)u = fe + (h^2/2 * A^2)x + (h^2/2 * A*B)*u
    let h2_2 = dt * dt / 2.0;
    let md_expected = fe_expected
        + (h2_2 * &state_matrix * &state_matrix) * dv_state0.clone()
        + (h2_2 * &state_matrix * &control_matrix) * dv_input0.clone();
    let error: f64 = (LtiState::<3, 0>::from_vec(md_result.to_vec())
        - LtiState::<3, 0>::from_vec(md_expected.as_slice().to_vec()))
    .to_vec()
    .iter()
    .map(|x| x * x)
    .sum();
    assert!(error < tol);

    let rk4_result = rk4.step(&model, &state0, Some(&input0), dt).unwrap();
    // r = (I + Ah + h^2/2 * A^2 + h^3/6 * A^3 + h^4/24 * A^4)x + h(I + h/2*A + h^2/6 * A^2 + h^3/24 * A^3)*B*u =
    //   md + (h^3/6 * A^3 + h^4/24 * A^4)x +   h(h^2/6*A^2 + h^3/24 * A^3)Bu
    let h2_6 = dt * dt / 6.0;
    let h3_6 = dt * h2_6;
    let h3_24 = h3_6 / 4.0;
    let h4_24 = dt * h3_24;
    let sm_2 = &state_matrix * &state_matrix;
    let sm_3 = &state_matrix * &sm_2;
    let sm_4 = &state_matrix * &sm_3;

    let rk4_expected = md_expected
        + (h3_6 * &sm_3 + h4_24 * &sm_4) * dv_state0
        + dt * (h2_6 * &sm_2 + h3_24 * &sm_3) * &control_matrix * dv_input0;
    let error: f64 = (LtiState::<3, 0>::from_vec(rk4_result.to_vec())
        - LtiState::<3, 0>::from_vec(rk4_expected.as_slice().to_vec()))
    .to_vec()
    .iter()
    .map(|x| x * x)
    .sum();
    assert!(error < tol);
}

#[test]
fn test_rk4_vs_zoh() {
    let state_matrix = DMatrix::from_vec(3, 3, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0]);
    let control_matrix = DMatrix::from_vec(3, 2, vec![1.0, 2.0, 0.0, 1.0, 2.0, 1.0]);
    let model = LtiModel::<3, 0, 2>::new(state_matrix, control_matrix).unwrap();

    let tol = 1e-5;
    let dt = 0.01;

    let state0 = LtiState::<3, 0>::new([3.0, 2.0, 1.0]);
    let input0 = LtiInput::<2, 0>::new([1.0, 1.0]);

    let rk4 = RK4::new(&model).unwrap();
    let zoh = ZOH::new(&model, dt).unwrap();

    let rk4_result = rk4.step(&model, &state0, Some(&input0), dt).unwrap();
    let zoh_result = zoh.step(&model, &state0, Some(&input0), dt).unwrap();

    let error: f64 = (LtiState::<3, 0>::from_vec(rk4_result.to_vec())
        - LtiState::<3, 0>::from_vec(zoh_result.to_vec()))
    .to_vec()
    .iter()
    .map(|x| x * x)
    .sum();

    assert!(error < tol);
}
