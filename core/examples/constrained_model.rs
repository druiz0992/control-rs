#![allow(unused_imports)]
use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::numeric_services::solver::OptimizerConfig;
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::physics::discretizer::{
    BackwardEuler, Discretizer, ForwardEuler, HermiteSimpson, ImplicitMidpoint, MidPoint, RK4,
    RK4Symbolic,
};
use control_rs::physics::models::{
    BouncingBall, BouncingBallState, DoublePendulum, DoublePendulumState,
};
use control_rs::physics::simulator::{BasicSim, PhysicsSim};
use control_rs::physics::traits::Dynamics;
use control_rs::{plotter, utils::helpers};
use std::f64::consts::PI;
use std::io::{self, Write};
use std::sync::Arc;

#[derive(Debug, Clone)]
enum IntegratorType {
    BackwardEuler,
    ImplicitMidpoint,
}

async fn build_sim(integrator: IntegratorType) {
    let dt = 0.01;
    let steps = 500;

    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 5.0;
    let v_x = 1.0;
    let v_y = 7.5;

    let registry = Arc::new(ExprRegistry::new());
    let state0 = BouncingBallState::new(pos_x, pos_y, v_x, v_y);

    let model = BouncingBall::new(m, friction_coeff, Some(&registry));

    let states = match integrator {
        IntegratorType::BackwardEuler => {
            let integrator = BackwardEuler::new(&model, Arc::clone(&registry), None).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
        IntegratorType::ImplicitMidpoint => {
            let solver_options = OptimizerConfig::default()
                .set_verbose(true)
                // TODO - review why this simulation is wrong
                .set_tolerance(1e20)
                .unwrap();
            let integrator =
                ImplicitMidpoint::new(&model, Arc::clone(&registry), Some(solver_options)).unwrap();
            let sim = BasicSim::new(model.clone(), integrator);
            sim.rollout(&state0, None, dt, steps).unwrap()
        }
    };

    let animation = Macroquad::new();
    animation
        .run_animation(&model, &states, (400.0, 300.0))
        .await
        .unwrap();
    let times: Vec<_> = (0..states.len()).map(|i| i as f64 * dt).collect();
    let energy: Vec<_> = states.iter().filter_map(|s| model.energy(s)).collect();

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energy, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}

#[macroquad::main("Physics Bouncing Ball")]
async fn main() {
    let integrator_type = vec![
        IntegratorType::BackwardEuler,
        IntegratorType::ImplicitMidpoint,
    ];
    env_logger::init();

    for integrator in integrator_type {
        println!("Discretizer: {:?}", integrator);
        build_sim(integrator).await;
        println!("Press Enter to continue...");
        let _ = io::stdout().flush();
        let _ = io::stdin().read_line(&mut String::new());
    }
}
