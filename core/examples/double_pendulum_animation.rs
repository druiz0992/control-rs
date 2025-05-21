use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::physics::discretizer::rk4::RK4;
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::simulator::BasicSim;
use std::f64::consts::PI;

#[macroquad::main("Physics Double Pendulum")]
async fn main() {
    env_logger::init();
    let m1 = 1.0;
    let m2 = 1.0;
    let l1 = 1.0;
    let l2 = 1.0;
    let air_resistance_coeff = 0.2;

    let theta1 = PI;
    let omega1 = 0.0;
    let theta2 = PI - 0.1;
    let omega2 = 0.0;

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, None);
    let state0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);

    let integrator = RK4::new(&model).unwrap();
    let sim = BasicSim::new(model, integrator, state0);
    let animation_sim = Macroquad::new(sim);

    animation_sim.run_animation((400.0, 300.0), None).await;
}
