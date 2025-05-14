#![allow(unused_imports)]
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::physics::discretizer::{
    BackwardEuler, Discretizer, HermiteSimpson, ImplicitMidpoint, RK4Symbolic,
};
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::simulator::{BasicSim, PhysicsSim};
use control_rs::numeric_services::solver::OptimizerConfig;
use control_rs::{plotter, utils};
use std::f64::consts::PI;
use std::sync::Arc;

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

    let registry = Arc::new(ExprRegistry::new());
    let state0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);

    let dt = 0.01;
    let steps = 1000;

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
    let mut solver_options = OptimizerConfig::default();
    solver_options.set_verbose(true);
    //let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();
    //let integrator = BackwardEuler::new(model, Arc::clone(&registry)).unwrap();
    //let integrator = HermiteSimpson::new(model, Arc::clone(&registry)).unwrap();
    let integrator = ImplicitMidpoint::new(model, Arc::clone(&registry), Some(solver_options)).unwrap();
    let mut sim = BasicSim::new(integrator, state0);

    let history = sim.simulate_steps(dt, steps);

    let (times, states, energies) = utils::unzip3(history);

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energies, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}
