use control_rs::physics::discretizer::RK4;
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::simulator::{BasicSim, PhysicsSim};
use control_rs::physics::traits::Dynamics;
use control_rs::plotter;
use std::f64::consts::PI;

fn main() {
    env_logger::init();
    let m1 = 1.0;
    let m2 = 1.0;
    let l1 = 1.0;
    let l2 = 1.0;
    let air_resistance_coeff = 0.0;

    let theta1 = PI / 1.6;
    let omega1 = 0.0;
    let theta2 = PI / 1.8;
    let omega2 = 0.0;

    let state0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);

    let dt = 0.01;
    let steps = 1000;

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, None);
    let integrator = RK4::new(&model).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    let states = sim.rollout(&state0, None, dt, steps).unwrap();
    let times: Vec<_> = (0..states.len()).map(|i| i as f64 * dt).collect();
    let energy: Vec<_> = states.iter().filter_map(|s| model.energy(s)).collect();

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energy, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}
