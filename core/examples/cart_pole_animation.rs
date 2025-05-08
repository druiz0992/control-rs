use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::physics::discretizer::rk4::RK4;
use control_rs::physics::models::{CartPole, CartPoleState};
use control_rs::physics::simulator::BasicSim;
use std::f64::consts::PI;

#[macroquad::main("Physics Double Pendulum")]
async fn main() {
    let m = 1.0;
    let cart_mass = 2.0;
    let l = 1.0;
    let friction_coeff = 1.0;
    let air_resistance_coeff = 1.0;

    let pos_x = 0.0;
    let v_x = -1.0;
    let theta = PI / 1.8;
    let omega = 0.0;

    let model = CartPole::new(m, cart_mass, friction_coeff, air_resistance_coeff, l, None);
    let state0 = CartPoleState::new(pos_x, v_x, omega, theta);

    let integrator = RK4::new();
    let mut sim = BasicSim::new(model, integrator, state0);
    let animation_sim = Macroquad::<BasicSim<CartPole, RK4>>::new();

    animation_sim.run_animation(&mut sim, (400.0, 300.0)).await;
}
