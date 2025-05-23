use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::numeric_services::solver::OptimizerConfig;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::BackwardEuler;
use control_rs::physics::models::{BouncingBall, BouncingBallState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::PhysicsSim;
use std::sync::Arc;

#[macroquad::main("Physics Double Pendulum")]
async fn main() {
    env_logger::init();
    let dt = 0.05;
    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 5.0;
    let v_x = 1.0;
    let v_y = 7.5;

    let registry = Arc::new(ExprRegistry::new());
    let state0 = BouncingBallState::new(pos_x, pos_y, v_x, v_y);

    let model = BouncingBall::new(m, friction_coeff, Some(&registry));
    let mut solver_options = OptimizerConfig::default();
    solver_options.set_verbose(true);

    let integrator = BackwardEuler::new(&model, registry, Some(solver_options)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);
    let states = sim.rollout(&state0, None, dt, 1000).unwrap();

    let animation = Macroquad::new();
    animation
        .run_animation(&model, &states, (400.0, 300.0))
        .await
        .unwrap();
}
