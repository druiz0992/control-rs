use control_rs::common::Labelizable;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::{
    BackwardEuler, ForwardEuler, HermiteSimpson, ImplicitMidpoint, MidPoint, RK4, RK4Symbolic,
};
use control_rs::physics::models::{SlidingBrick, SlidingBrickState};
use control_rs::physics::traits::Discretizer;
use std::sync::Arc;

fn init_sliding_brick() -> (SlidingBrick, SlidingBrickState, Arc<ExprRegistry>) {
    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 1.0;
    let v_x = 1.0;
    let v_y = 4.5;

    let registry = Arc::new(ExprRegistry::new());
    let state = SlidingBrickState::new(pos_x, pos_y, v_x, v_y);

    let model = SlidingBrick::new(m, friction_coeff, Some(&registry));

    (model, state, registry)
}

#[test]
fn test_explicit_integrators_ko_with_constrained_dynamics() {
    let (model, _, registry) = init_sliding_brick();

    let rk4 = RK4::new(model.clone());
    let fe = ForwardEuler::new(model.clone());
    let mp = MidPoint::new(model.clone());
    let rk4_symbolic = RK4Symbolic::new(model, registry);

    assert!(rk4.is_err());
    assert!(rk4_symbolic.is_err());
    assert!(fe.is_err());
    assert!(mp.is_err());
}

#[test]
fn test_hermite_simpson_ko_with_constrained_dynamics() {
    let (model, _, registry) = init_sliding_brick();

    let hs = HermiteSimpson::new(model, registry, None);

    assert!(hs.is_err());
}

#[test]
fn test_implicit_ok_with_constrained_dynamics() {
    let (model, _, registry) = init_sliding_brick();

    let be = BackwardEuler::new(model.clone(), registry.clone(), None);
    let imp = ImplicitMidpoint::new(model, registry, None);

    assert!(be.is_ok());
    assert!(imp.is_ok());
}

#[test]
fn test_backward_euler_constrained_dynamics() {
    let (model, mut result, registry) = init_sliding_brick();

    let dt = 0.01;

    let mut integrator = BackwardEuler::new(model, registry, None).unwrap();

    for _ in 0..2000 {
        result = integrator.step(&result, None, dt).unwrap();
        let [pos_y] = result.extract(&["pos_y"]);

        // check constain is met
        assert!(pos_y >= 0.0);
    }
}

#[test]
fn test_implicit_midpoint_constrained_dynamics() {
    let (model, mut result, registry) = init_sliding_brick();

    let dt = 0.01;
    let tol = 1e-1;

    let mut integrator = ImplicitMidpoint::new(model, registry, None).unwrap();

    for _ in 0..2000 {
        result = integrator.step(&result, None, dt).unwrap();
        let [pos_y] = result.extract(&["pos_y"]);

        // check constain is met
        assert!(pos_y >= -tol);
    }
}
