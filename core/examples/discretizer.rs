#![allow(unused_imports)]
use control_rs::numeric_services::solver::OptimizerConfig;
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::physics::discretizer::{
    BackwardEuler, Discretizer, ForwardEuler, HermiteSimpson, ImplicitMidpoint, MidPoint, RK4,
    RK4Symbolic,
};
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::simulator::{BasicSim, PhysicsSim};
use control_rs::physics::traits::Dynamics;
use control_rs::{plotter, utils::helpers};
use std::f64::consts::PI;
use std::io::{self, Write};
use std::sync::Arc;

#[derive(Debug, Clone)]
enum IntegratorType {
    RK4Symbolic,
    RK4,
    BackwardEuler,
    HermiteSimpson,
    ForwardEuler,
    MidPoint,
    ImplicitMidpoint,
}

// Example iterates over all implemented discretizers and shows a plot of the energy of the model and the state trajectory.
// The model energy shows some of the qualities of the different discretizers used:
// - Forward Euler              : Tends to increase energy over time, as it extrapolates forward.
// - Backward Euler (Implicit)  : Dissipates energy,
// - MidPoint                   : Second order method. It doesnt conserve energy, but ofted oscillates around true energy
// - MidPoint (Implicit)        : Approximately conserves energy over long time
// - RK4                        : Doesn't conserve energy over long time, but has low error per step.
// - Hermit Simpson (Implicit)  : Can conserve energy well

fn build_sim(integrator: IntegratorType) {
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

    let states = match integrator {
        IntegratorType::BackwardEuler => {
            let integrator = BackwardEuler::new(&model, Arc::clone(&registry), None).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::HermiteSimpson => {
            let integrator = HermiteSimpson::new(&model, Arc::clone(&registry), None).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::ImplicitMidpoint => {
            let solver_options = OptimizerConfig::default().set_verbose(true);
            let integrator =
                ImplicitMidpoint::new(&model, Arc::clone(&registry), Some(solver_options)).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::RK4Symbolic => {
            let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::RK4 => {
            let integrator = RK4::new(&model).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::MidPoint => {
            let integrator = MidPoint::new(&model).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::ForwardEuler => {
            let integrator = ForwardEuler::new(&model).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
    };

    let times: Vec<_> = (0..states.len()).map(|i| i as f64 * dt).collect();
    let energy: Vec<_> = states.iter().filter_map(|s| model.energy(s)).collect();

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energy, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}

fn main() {
    let integrator_type = vec![
        IntegratorType::RK4Symbolic,
        IntegratorType::RK4,
        IntegratorType::BackwardEuler,
        IntegratorType::HermiteSimpson,
        IntegratorType::ForwardEuler,
        IntegratorType::MidPoint,
        IntegratorType::ImplicitMidpoint,
    ];
    env_logger::init();

    for integrator in integrator_type {
        println!("Discretizer: {:?}", integrator);
        build_sim(integrator);
        println!("Press Enter to continue...");
        let _ = io::stdout().flush();
        let _ = io::stdin().read_line(&mut String::new());
    }
}
