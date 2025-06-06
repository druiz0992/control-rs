use control_rs::controllers::qp_lqr::options::QPOptions;
use control_rs::controllers::qp_mpc::linear::ConvexMpc;
use control_rs::controllers::qp_mpc::options::ConvexMpcOptions;
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::{
    ConstraintTransform, Controller, ControllerOptions, IndirectShootingLQR, QPLQR,
    RiccatiRecursionLQR,
};
use control_rs::cost::generic::{GenericCost, GenericCostOptions};
use control_rs::physics::discretizer::ZOH;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::plotter;
use nalgebra::{DMatrix, dmatrix};
use osqp::Settings;
use std::io::{self, Write};

#[derive(Debug, Clone)]
enum LinearControllerType {
    IndirectShootingLqr,
    QpLqr,
    QpLqrUlimits(f64, f64),
    RiccatiRecursionLQRFinite,
    RiccatiRecursionLQRInfinite,
    RiccatiRecursionLQRFiniteULimitsAndNoise(f64, f64, f64, f64),
    RiccatiRecursionLQRInfiniteULimitsAndNoise(f64, f64, f64, f64),
    MpcLinear,
    MpcLinearULimitsAndNoise(f64, f64, f64, f64),
}

// Example iterates over all implemented linear controllers

type LtiSim = BasicSim<LtiModel<2, 0, 1>, ZOH<LtiModel<2, 0, 1>>>;
type LinearContoller = Box<dyn Controller<LtiSim>>;

fn build_sim(controller_type: LinearControllerType) {
    let state_matrix = dmatrix![0.0,1.0; 0.0,0.0];
    let control_matrix = dmatrix![0.0; 1.0];
    let model = LtiModel::<2, 0, 1>::new(state_matrix, control_matrix).unwrap();

    let initial_state = LtiState::<2, 0>::new([1.0, 0.0]);
    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let integrator = ZOH::new(&model, dt).unwrap();

    let sim = BasicSim::new(model.clone(), integrator, None);

    let q_matrix = DMatrix::<f64>::identity(2, 2);
    let qn_matrix = DMatrix::<f64>::identity(2, 2);
    let r_matrix = DMatrix::<f64>::identity(1, 1) * 0.1;
    let expected_trajectory: Vec<_> = (0..n_steps).map(|_| LtiState::default()).collect();

    let options = GenericCostOptions::new().set_reference_state_trajectory(&expected_trajectory);
    let cost = GenericCost::<_, LtiInput<1, 0>>::new(q_matrix, qn_matrix, r_matrix, Some(options))
        .unwrap();

    let general_options = ControllerOptions::<LtiSim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(sim_time)
        .unwrap();
    let mut controller: LinearContoller = match controller_type {
        LinearControllerType::IndirectShootingLqr => Box::new(
            IndirectShootingLQR::new(sim, Box::new(cost.clone()), Some(general_options)).unwrap(),
        ),
        LinearControllerType::QpLqr => {
            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQR::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                Some(qp_options),
            )
            .unwrap();
            Box::new(controller)
        }
        LinearControllerType::QpLqrUlimits(lower, upper) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((lower, upper));
            let general_options = general_options.set_u_limits(constraints);
            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::<LtiSim>::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQR::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                Some(qp_options),
            )
            .unwrap();
            Box::new(controller)
        }
        LinearControllerType::RiccatiRecursionLQRFinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), Some(options)).unwrap())
        }

        LinearControllerType::RiccatiRecursionLQRInfinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), Some(options)).unwrap())
        }
        LinearControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(
            lower,
            upper,
            std_0,
            std_n,
        ) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((lower, upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);
            Box::new(RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), Some(options)).unwrap())
        }
        LinearControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(
            lower,
            upper,
            std_0,
            std_n,
        ) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((lower, upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), Some(options)).unwrap())
        }
        LinearControllerType::MpcLinear => {
            let osqp_settings = Settings::default().verbose(false).eps_abs(1e-7);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpc::new(sim, Box::new(cost.clone()), &initial_state, Some(options)).unwrap(),
            )
        }
        LinearControllerType::MpcLinearULimitsAndNoise(lower, upper, std_0, std_n) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((lower, upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let osqp_settings = Settings::default().verbose(false).eps_abs(1e-7);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpc::new(sim, Box::new(cost.clone()), &initial_state, Some(options)).unwrap(),
            )
        }
    };

    let (x_traj, u_traj) = controller.solve(&initial_state).unwrap();
    let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();

    plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
    plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}

fn main() {
    let controller_type = vec![
        LinearControllerType::IndirectShootingLqr,
        LinearControllerType::QpLqr,
        LinearControllerType::QpLqrUlimits(-0.5, 0.5),
        LinearControllerType::RiccatiRecursionLQRFinite,
        LinearControllerType::RiccatiRecursionLQRInfinite,
        LinearControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(-4.5, 4.5, 10.0, 0.1),
        LinearControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(-4.5, 4.5, 10.0, 0.1),
        LinearControllerType::MpcLinear,
        LinearControllerType::MpcLinearULimitsAndNoise(-19.5, 19.5, 10.0, 0.1),
    ];
    env_logger::init();

    for controller in controller_type {
        println!("Controller: {:?}", controller);
        build_sim(controller);
        println!("Press Enter to continue...");
        let _ = io::stdout().flush();
        let _ = io::stdin().read_line(&mut String::new());
    }
}
