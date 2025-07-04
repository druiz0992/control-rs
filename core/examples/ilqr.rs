use control_rs::animation::{Animation, macroquad::Macroquad};
use control_rs::controllers::ddp::DDPOptions;
use control_rs::controllers::ddp::controller::DDP;
use control_rs::controllers::{Controller, ControllerOptions};
use control_rs::cost::GenericCostOptions;
use control_rs::cost::generic::GenericCost;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Numeric;
use control_rs::physics::models::{Quadrotor2D, Quadrotor2DInput, Quadrotor2DState};
use control_rs::physics::simulator::BasicSim;
use control_rs::plotter;
use nalgebra::DMatrix;
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;
use std::io::{self, Write};
use std::sync::Arc;
use symbolic_services::symbolic::ExprRegistry;

#[derive(Debug, Clone)]
enum ControllerType {
    Ilqr,
    Ddp,
}

// Example iterates over all implemented linear controllers

type Sim<M> = BasicSim<M, RK4Numeric<M>>;
type SymbolicController<M> = Box<dyn Controller<Sim<M>>>;

async fn build_sim(controller_type: ControllerType) {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let state_0 = Quadrotor2DState::new(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    let state_goal = Quadrotor2DState::new(0.0, 1.0, PI, 0.0, 0.0, 0.0);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));

    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let df_du = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_jacobian_u);
    let df_dx = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_jacobian_x);
    let d2f_dxx = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_d2f_dxx);
    let d2f_dxu = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_d2f_dxu);
    let d2f_dux = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_d2f_dux);
    let d2f_duu = Arc::new(ffi_codegen::rk4::quadrotor_2d::rk4_quadrotor_2d_d2f_duu);

    let integrator = RK4Numeric::new(
        &model,
        df_dx,
        df_du,
        Some((d2f_dxx, d2f_dxu, d2f_dux, d2f_duu)),
    )
    .unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6) * 1.0;
    let qn_matrix = DMatrix::<f64>::identity(6, 6) * 100.0;
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    // initialize u_ref with random noise
    let normal = Normal::new(0.0, 1.0).unwrap();
    let mut rng = rand::thread_rng();
    let u_ref: Vec<_> = (0..n_steps - 1)
        .map(|_| {
            let u1 = normal.sample(&mut rng);
            let u2 = normal.sample(&mut rng);
            Quadrotor2DInput::new(u1, u2)
        })
        .collect();

    let options = GenericCostOptions::<Quadrotor2DState, Quadrotor2DInput>::new()
        .set_reference_state_trajectory(&vec![state_goal.clone(); n_steps]);
    let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();

    let general_options = ControllerOptions::<Sim<Quadrotor2D>>::default()
        .set_x_ref(&[state_goal.clone()])
        .set_u_ref(&u_ref)
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(sim_time)
        .unwrap();

    let mut controller: SymbolicController<Quadrotor2D> = match controller_type {
        ControllerType::Ilqr => {
            let ilqr_options = DDPOptions::<Sim<Quadrotor2D>>::default()
                .set_general(general_options)
                .set_verbose(true).set_ilqr_enable(true);
            let controller = DDP::new_numeric(sim, Box::new(cost.clone()), ilqr_options).unwrap();
            Box::new(controller)
        }
        ControllerType::Ddp => {
            let ddp_options = DDPOptions::<Sim<Quadrotor2D>>::default()
                .set_general(general_options)
                .set_verbose(true);
            let controller = DDP::new_numeric(sim, Box::new(cost.clone()), ddp_options).unwrap();
            Box::new(controller)
        }
    };

    let (x_traj, u_traj) = controller.solve(&state_0).unwrap();
    let animation = Macroquad::new();

    animation
        .run_animation(&model, &x_traj, (400.0, 300.0))
        .await
        .unwrap();

    let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();

    plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
    plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}

#[macroquad::main("Physics Quadrotor")]
async fn main() {
    let m = 1.0;
    let u_limits = (0.2 * m * c::GRAVITY, 0.6 * m * c::GRAVITY);
    let controller_type = vec![ControllerType::Ilqr, ControllerType::Ddp];
    env_logger::init();

    for controller in controller_type {
        println!("Controller: {:?}", controller);
        build_sim(controller).await;
        println!("Press Enter to continue...");
        let _ = io::stdout().flush();
        let _ = io::stdin().read_line(&mut String::new());
    }
}
