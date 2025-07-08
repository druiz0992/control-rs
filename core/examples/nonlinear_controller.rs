use control_rs::animation::{Animation, macroquad::Macroquad};
use control_rs::controllers::qp_lqr::{QPLQR, QPOptions};
use control_rs::controllers::qp_mpc::{ConvexMpc, ConvexMpcOptions};
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursion};
use control_rs::controllers::{ConstraintAffine, Controller, ControllerOptions};
use control_rs::cost::generic::GenericCost;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::{Quadrotor2D, Quadrotor2DInput, Quadrotor2DState};
use control_rs::physics::simulator::BasicSim;
use control_rs::plotter;
use control_rs::utils::Labelizable;
use nalgebra::DMatrix;
use osqp::Settings;
use std::io::{self, Write};
use std::sync::Arc;
use symbolic_services::symbolic::ExprRegistry;

#[derive(Debug, Clone)]
enum ControllerType {
    QpLqr,
    QpLqrUlimits(f64, f64),
    RiccatiRecursionLQRFinite,
    RiccatiRecursionLQRInfinite,
    RiccatiRecursionLQRFiniteULimitsAndNoise(f64, f64, Vec<f64>),
    RiccatiRecursionLQRInfiniteULimitsAndNoise(f64, f64, Vec<f64>),
    Mpc,
    MpcULimitsAndNoise(f64, f64, Vec<f64>),
    MpcUXLimitsAndNoise(f64, f64, f64, f64, Vec<f64>),
}

// Example iterates over all implemented linear controllers

type Sim<M> = BasicSim<M, RK4Symbolic<M>>;
type SymbolicController<M> = Box<dyn Controller<Sim<M>>>;

async fn build_sim(controller_type: ControllerType) {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    // linearization points
    let input_hover = Quadrotor2DInput::new(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY);
    let state_hover = Quadrotor2DState::default();

    let state_0 = Quadrotor2DState::new(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    let state_ref = Quadrotor2DState::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));

    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6) * 1.0;
    let qn_matrix = DMatrix::<f64>::identity(6, 6) * 1.0;
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let mut reference_traj: Vec<_> = (0..n_steps + 1)
        .map(|_| Quadrotor2DState::default())
        .collect();
    reference_traj[n_steps] = state_ref.clone();

    //let options = GenericCostOptions::new().set_linear_term(true);
    let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, None).unwrap();

    let general_options = ControllerOptions::<Sim<Quadrotor2D>>::default()
        .set_x_ref(&[state_ref.clone()])
        .set_u_ref(&[input_hover.clone()])
        .set_x_operating(&[state_hover.clone()])
        .set_u_operating(&[input_hover.clone()])
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(sim_time)
        .unwrap();

    let mut controller: SymbolicController<Quadrotor2D> = match controller_type {
        ControllerType::QpLqr => {
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let qp_options = QPOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) =
                QPLQR::new_symbolic(sim, Box::new(cost.clone()), &state_0, Some(qp_options))
                    .unwrap();
            Box::new(controller)
        }
        ControllerType::QpLqrUlimits(lower_u, upper_u) => {
            let [hover] = input_hover.extract(&["u1"]);
            let input_constraints = ConstraintAffine::new_uniform_bounds_input::<Sim<Quadrotor2D>>(
                (lower_u - hover, upper_u - hover),
            );
            let general_options = general_options.set_u_limits(input_constraints);
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let qp_options = QPOptions::<Sim<Quadrotor2D>>::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) =
                QPLQR::new_symbolic(sim, Box::new(cost.clone()), &state_0, Some(qp_options))
                    .unwrap();
            Box::new(controller)
        }
        ControllerType::RiccatiRecursionLQRFinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursion::new_symbolic(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }

        ControllerType::RiccatiRecursionLQRInfinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursion::new_symbolic(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        ControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(lower, upper, std) => {
            let constraints =
                ConstraintAffine::new_uniform_bounds_input::<Sim<Quadrotor2D>>((lower, upper));
            let general_options = general_options.set_u_limits(constraints).set_noise(std);
            let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursion::new_symbolic(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        ControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(lower, upper, std) => {
            let constraints =
                ConstraintAffine::new_uniform_bounds_input::<Sim<Quadrotor2D>>((lower, upper));
            let general_options = general_options.set_u_limits(constraints).set_noise(std);
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursion::new_symbolic(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        ControllerType::Mpc => {
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpc::new_symbolic(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
        }
        ControllerType::MpcULimitsAndNoise(lower, upper, std) => {
            let [hover] = input_hover.extract(&["u1"]);
            let input_constraints = ConstraintAffine::new_uniform_bounds_input::<Sim<Quadrotor2D>>(
                (lower - hover, upper - hover),
            );

            let general_options = general_options
                .set_u_limits(input_constraints)
                .set_noise(std);
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpc::new_symbolic(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
        }
        ControllerType::MpcUXLimitsAndNoise(lower_u, upper_u, lower_x, upper_x, std) => {
            let [hover] = input_hover.extract(&["u1"]);
            let input_constraints = ConstraintAffine::new_uniform_bounds_input::<Sim<Quadrotor2D>>(
                (lower_u - hover, upper_u - hover),
            );

            let state_constraints =
                ConstraintAffine::new_single_bound_state::<Sim<Quadrotor2D>>(
                    (lower_x, upper_x),
                    2,
                )
                .unwrap();
            let general_options = general_options
                .set_u_limits(input_constraints)
                .set_x_limits(state_constraints)
                .set_noise(std);
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpc::new_symbolic(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
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
    let controller_type = vec![
        ControllerType::QpLqr,
        ControllerType::QpLqrUlimits(u_limits.0, u_limits.1),
        ControllerType::RiccatiRecursionLQRFinite,
        ControllerType::RiccatiRecursionLQRInfinite,
        ControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(
            u_limits.0,
            u_limits.1,
            vec![0.01; 6],
        ),
        ControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(
            u_limits.0,
            u_limits.1,
            vec![0.01; 6],
        ),
        ControllerType::Mpc,
        ControllerType::MpcULimitsAndNoise(u_limits.0, u_limits.1, vec![0.01; 6]),
        ControllerType::MpcUXLimitsAndNoise(u_limits.0, u_limits.1, -0.2, 0.2, vec![0.01; 6]),
    ];
    env_logger::init();

    for controller in controller_type {
        println!("Controller: {:?}", controller);
        build_sim(controller).await;
        println!("Press Enter to continue...");
        let _ = io::stdout().flush();
        let _ = io::stdin().read_line(&mut String::new());
    }
}
