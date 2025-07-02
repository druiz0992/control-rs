use control_rs::controllers::qp_lqr::options::QPOptions;
use control_rs::controllers::qp_mpc::ConvexMpc;
use control_rs::controllers::qp_mpc::options::ConvexMpcOptions;
use control_rs::controllers::riccati_lqr::RiccatiRecursion;
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::{
    ConstraintTransform, Controller, ControllerOptions, IndirectShooting, QPLQR,
};
use control_rs::cost::generic::{GenericCost, GenericCostOptions};
use control_rs::physics::discretizer::ZOH;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::State;
use nalgebra::{DMatrix, dmatrix, dvector};
use osqp::Settings;

enum LinearControllerType {
    IndirectShootingLqr,
    QpLqr,
    QpLqrUlimits(f64, f64),
    RiccatiRecursionLQRFinite,
    RiccatiRecursionLQRInfinite,
    MpcLinear,
    MpcLinearULimitsAndNoise(f64, f64, Vec<f64>),
}

type LtiSim = BasicSim<LtiModel<2, 0, 1>, ZOH<LtiModel<2, 0, 1>>>;
type LinearContoller = Box<dyn Controller<LtiSim>>;

fn linear_controller_setup(controller_type: LinearControllerType) {
    let state_matrix = dmatrix![0.0,1.0; 0.0,0.0];
    let control_matrix = dmatrix![0.0; 1.0];
    let model = LtiModel::<2, 0, 1>::new(state_matrix, control_matrix).unwrap();

    let initial_state = LtiState::<2, 0>::new([1.0, 0.0]);
    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let integrator = ZOH::new(&model, dt).unwrap();

    let sim = BasicSim::new(model.clone(), integrator);

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
    let mut controller: LinearContoller = match &controller_type {
        LinearControllerType::IndirectShootingLqr => Box::new(
            IndirectShooting::new_linear(sim, Box::new(cost.clone()), Some(general_options))
                .unwrap(),
        ),
        LinearControllerType::QpLqr => {
            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQR::new_linear(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                Some(qp_options),
            )
            .unwrap();
            Box::new(controller)
        }
        LinearControllerType::QpLqrUlimits(lower, upper) => {
            let contraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((*lower, *upper));
            let general_options = general_options.set_u_limits(contraints);
            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::<LtiSim>::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQR::new_linear(
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
            Box::new(
                RiccatiRecursion::new_linear(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }

        LinearControllerType::RiccatiRecursionLQRInfinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursion::new_linear(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        LinearControllerType::MpcLinear => {
            let osqp_settings = Settings::default().verbose(false).eps_abs(1e-7);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings)
                .set_apply_steady_state_cost(true);
            Box::new(
                ConvexMpc::new_linear(sim, Box::new(cost.clone()), &initial_state, Some(options))
                    .unwrap(),
            )
        }
        LinearControllerType::MpcLinearULimitsAndNoise(lower, upper, std) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<LtiSim>((*lower, *upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise(std.clone());
            let osqp_settings = Settings::default().verbose(false).eps_abs(1e-7);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings)
                .set_apply_steady_state_cost(true);
            Box::new(
                ConvexMpc::new_linear(sim, Box::new(cost.clone()), &initial_state, Some(options))
                    .unwrap(),
            )
        }
    };

    let (x_traj, u_traj) = controller.solve(&initial_state).unwrap();

    let tol = 1e-2;
    assert!(
        (x_traj.last().unwrap().to_vector() - expected_trajectory.last().unwrap().to_vector())
            .abs()
            .sum()
            < tol
    );
    assert!(
        (u_traj.last().unwrap().to_vector() - dvector!(0.0))
            .abs()
            .sum()
            < tol
    );

    if let LinearControllerType::QpLqrUlimits(lower, upper) = controller_type {
        let exceed_limits: Vec<_> = u_traj
            .iter()
            .filter(|u| u.to_vec()[0] < lower - tol || u.to_vec()[0] > upper + tol)
            .collect();
        assert!(exceed_limits.is_empty());
    }
}

#[test]
fn indirect_linear() {
    linear_controller_setup(LinearControllerType::IndirectShootingLqr);
}

#[test]
fn qp_lqr_linear() {
    linear_controller_setup(LinearControllerType::QpLqr);
}

#[test]
fn test_qp_lqr_linear_ulimits() {
    linear_controller_setup(LinearControllerType::QpLqrUlimits(-0.5, 0.5));
}

#[test]
fn test_ricatti_linear_infinite() {
    linear_controller_setup(LinearControllerType::RiccatiRecursionLQRInfinite);
}

#[test]
fn test_ricatti_linear_finite() {
    linear_controller_setup(LinearControllerType::RiccatiRecursionLQRFinite);
}

#[test]
fn test_mpc_linear() {
    linear_controller_setup(LinearControllerType::MpcLinear);
}

#[test]
fn test_mpc_noise_linear() {
    linear_controller_setup(LinearControllerType::MpcLinearULimitsAndNoise(
        -0.5,
        0.5,
        vec![0.0, 0.0],
    ));
}
