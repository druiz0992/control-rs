use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::physics::discretizer::rk4::RK4;
use control_rs::physics::models::{CartPole, CartPoleState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::PhysicsSim;
use std::f64::consts::PI;

// Simple example showing an animation of a discretized Cart Pole problem using RK4
#[macroquad::main("Cart Pole")]
async fn main() {
    env_logger::init();
    // Model parameters
    let m = 1.0;
    let cart_mass = 2.0;
    let l = 1.0;
    let friction_coeff = 1.0;
    let air_resistance_coeff = 1.0;

    // State
    let pos_x = 0.0;
    let v_x = -1.0;
    let theta = PI / 1.8;
    let omega = 0.0;

    // Initialize model and initial state
    let model = CartPole::new(
        m,
        cart_mass,
        l,
        friction_coeff,
        air_resistance_coeff,
        None,
        true,
    );
    let state0 = CartPoleState::new(pos_x, v_x, theta, omega);

    // Initialize RK4 discretizer
    let integrator = RK4::<CartPole>::new(&model).unwrap();
    // Initialize Simulator
    let sim = BasicSim::new(model.clone(), integrator, None);

    // Starting from initial state, rollout simulation for 500 ticks @ 100 Hz
    let states = sim.rollout(&state0, None, 0.01, 500).unwrap();

    Macroquad::new()
        .run_animation(&model, &states, (400.0, 300.0))
        .await
        .unwrap();
}
