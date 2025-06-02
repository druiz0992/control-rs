use control_rs::animation::{Animation, macroquad::Macroquad};
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::riccati_lqr::symbolic::RiccatiRecursionSymbolic;
use control_rs::controllers::{Controller, ControllerOptions};
use control_rs::cost::generic::GenericCost;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::{Quadrotor2D, Quadrotor2DInput, Quadrotor2DState};
use control_rs::physics::simulator::BasicSim;
use nalgebra::DMatrix;
use std::sync::Arc;

#[macroquad::main("Physics Quadrotor")]
async fn main() {
    env_logger::init();
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    let sim_time = 10.0;
    //let n_steps = (sim_time / dt) as usize + 1;

    //let u_limits = (0.2 * m * c::GRAVITY, 0.6 * m * c::GRAVITY);

    // linearization points
    let input_hover = Quadrotor2DInput::new(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY);
    let state_hover = Quadrotor2DState::default();

    let state_0 = Quadrotor2DState::new(6.0, 20.0, 0.0, 0.0, 0.0, 0.0);
    let state_ref = Quadrotor2DState::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6);
    let qn_matrix = DMatrix::<f64>::identity(6, 6);
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, None).unwrap();

    let general_options = ControllerOptions::<BasicSim<Quadrotor2D, RK4Symbolic<_>>>::default()
        .set_x_ref(&[state_ref])
        .set_u_operating(&input_hover)
        .set_x_operating(&state_hover);
    let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);

    let mut lqr_riccati =
        RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), sim_time, dt, Some(options))
            .unwrap();

    let (x_traj_riccati, _) = lqr_riccati.solve(&state_0).unwrap();
    let animation = Macroquad::new();

    animation
        .run_animation(&model, &x_traj_riccati, (400.0, 300.0))
        .await
        .unwrap();
}
