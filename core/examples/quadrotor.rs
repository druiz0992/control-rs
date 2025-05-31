use control_rs::animation::{Animation, macroquad::Macroquad};
use control_rs::controllers::qp_lqr::symbolic::QPLQRSymbolic;
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::riccati_lqr::symbolic::RiccatiRecursionSymbolic;
use control_rs::controllers::{Controller, ControllerOptions};
use control_rs::cost::generic::{GenericCost, GenericCostOptions};
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::{Quadrotor2D, Quadrotor2DInput, Quadrotor2DState};
use control_rs::physics::simulator::BasicSim;
use control_rs::plotter;
use nalgebra::DMatrix;
use std::sync::Arc;

fn main() {
    env_logger::init();
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let u_limits = (0.2 * m * c::GRAVITY, 0.6 * m * c::GRAVITY);

    // linearization points
    let input_hover = Quadrotor2DInput::new(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY);
    let state_hover = Quadrotor2DState::default();

    let state_0 = Quadrotor2DState::new(2.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    let state_ref = Quadrotor2DState::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6) * 4.0;
    let qn_matrix = DMatrix::<f64>::identity(6, 6) * 10.0;
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let mut reference_traj: Vec<_> = (0..n_steps + 1)
        .map(|_| Quadrotor2DState::default())
        .collect();
    reference_traj[n_steps] = state_ref.clone();

    /*
    let options = GenericCostOptions::new().set_reference_state_trajectory(&reference_traj);
    let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();
    */
    let options = GenericCostOptions::new().set_linear_term(true);
    let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();

    let general_options = ControllerOptions::<BasicSim<Quadrotor2D, RK4Symbolic<_>>>::default()
        .set_x_ref(&vec![state_ref.clone()])
        .set_u_operating(&input_hover)
        .set_x_operating(&state_hover);
    //options.set_u_limits((-0.5, 0.1));
    let (mut controller, _) = QPLQRSymbolic::new(
        sim,
        Box::new(cost.clone()),
        &state_0,
        sim_time,
        dt,
        Some(general_options),
    )
    .unwrap();
    /*
        let mut options = RiccatiLQROptions::enable_infinite_horizon();
        options.general.set_x_ref(&state_ref);
        options.general.set_u_equilibrium(&input_hover);
        options.general.set_x_equilibrium(&state_hover);
        options.general.set_u_limits(u_limits);

    let mut controller =
        RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), sim_time, dt, Some(options))
            .unwrap();

    */
    controller.solve(&state_0).unwrap();
    let x_traj = controller.rollout(&state_0).unwrap();
    let u_traj = controller.get_u_traj();

    let times: Vec<_> = (0..x_traj.len()).map(|i| i as f64 * dt).collect();

    plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot1.png").unwrap();

    plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}
