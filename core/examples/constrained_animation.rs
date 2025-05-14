use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::BackwardEuler;
use control_rs::physics::models::{SlidingBrick, SlidingBrickState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::PhysicsSim;
use control_rs::{plotter, utils};
use std::sync::Arc;

#[macroquad::main("Physics Double Pendulum")]
async fn main() {
    env_logger::init();
    let m = 1.0;
    let friction_coeff = 0.0;

    let pos_x = 0.0;
    let pos_y = 15.0;
    let v_x = 3.0;
    let v_y = 4.5;

    let registry = Arc::new(ExprRegistry::new());
    let state0 = SlidingBrickState::new(pos_x, pos_y, v_x, v_y);

    let model = SlidingBrick::new(m, friction_coeff, Some(&registry));
    let integrator = BackwardEuler::new(model, registry).unwrap();
    let mut sim = BasicSim::new(integrator, state0);

    /*
    let dt = 0.01;
    let steps = 1000;
    let history = sim.simulate_steps(dt, steps);

    let (times, states, energies) = utils::unzip3(history);

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energies, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
    */

    let animation_sim = Macroquad::new(sim);
    animation_sim.run_animation((400.0, 300.0)).await;
}
