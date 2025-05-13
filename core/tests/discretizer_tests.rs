use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::BackwardEuler;
use control_rs::physics::models::{SlidingBrick, SlidingBrickState};
use control_rs::physics::traits::Discretizer;
use std::sync::Arc;

#[test]
fn test_constrained_dynamics() {
    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 1.0;
    let v_x = 1.0;
    let v_y = 4.5;

    let registry = Arc::new(ExprRegistry::new());
    let state0 = SlidingBrickState::new(pos_x, pos_y, v_x, v_y);

    let dt = 0.01;

    let model = SlidingBrick::new(m, friction_coeff, Some(&registry));
    let mut integrator = BackwardEuler::new(model, registry).unwrap();

    let result = integrator.step(&state0, None, dt).unwrap();
    dbg!(&result);
}
