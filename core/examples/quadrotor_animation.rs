use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::physics::discretizer::rk4::RK4;
use control_rs::physics::models::quadrotor_2d::model::Quadrotor2D;
use control_rs::physics::models::quadrotor_2d::state::Quadrotor2DState;
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::PhysicsSim;
use std::f64::consts::PI;

#[macroquad::main("Physics Quadrotos")]
async fn main() {
    env_logger::init();
    let m = 1.0;
    let j = 1.0;
    let l = 1.0;

    let pos_x = 0.0;
    let pos_y = 10.0;
    let theta = PI / 2.0;
    let v_x = 0.0;
    let v_y = 0.0;
    let omega = 0.0;

    let model = Quadrotor2D::new(m, j, l, None);
    let state0 = Quadrotor2DState::new(pos_x, pos_y, theta, v_x, v_y, omega);

    let integrator = RK4::new(&model).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);
    let states = sim.rollout(&state0, None, 0.001, 10000).unwrap();
    let animation = Macroquad::new();

    animation
        .run_animation(&model, &states, (400.0, 300.0))
        .await
        .unwrap();
}
