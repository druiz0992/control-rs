use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::{
    Controller, ControllerOptions, IndirectShootingLQR, QPLQR, RiccatiRecursionLQR,
};
use control_rs::cost::generic::{GenericCost, GenericCostOptions};
use control_rs::physics::discretizer::ZOH;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::State;
use nalgebra::{DMatrix, dmatrix, dvector};

enum LinearControllerType {
    IndirectShootingLQR,
    QPLQR,
    QPLQRUlimits(f64, f64),
    RiccatiRecursionLQRFinite,
    RiccatiRecursionLQRInfinite,
}

type LinearContoller = Box<dyn Controller<BasicSim<LtiModel<2, 0, 1>, ZOH<LtiModel<2, 0, 1>>>>>;

fn linear_controller_setup(controller_type: LinearControllerType) {
    let state_matrix = dmatrix![0.0,1.0; 0.0,0.0];
    let control_matrix = dmatrix![0.0; 1.0];
    let model = LtiModel::<2, 0, 1>::new(state_matrix, control_matrix).unwrap();

    let initial_state = LtiState::<2, 0>::new([1.0, 0.0]);
    let dt = 0.1;
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

    let mut controller: LinearContoller = match controller_type {
        LinearControllerType::IndirectShootingLQR => {
            Box::new(IndirectShootingLQR::new(sim, Box::new(cost.clone()), sim_time, dt).unwrap())
        }
        LinearControllerType::QPLQR => {
            let (controller, _) = QPLQR::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                sim_time,
                dt,
                None,
            )
            .unwrap();
            Box::new(controller)
        }
        LinearControllerType::QPLQRUlimits(lower, upper) => {
            let options = ControllerOptions::<BasicSim<LtiModel<2, 0, 1>, ZOH<_>>>::default()
                .set_u_limits((lower, upper));
            let (controller, _) = QPLQR::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                sim_time,
                dt,
                Some(options),
            )
            .unwrap();
            Box::new(controller)
        }
        LinearControllerType::RiccatiRecursionLQRFinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon();
            Box::new(
                RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), sim_time, dt, Some(options))
                    .unwrap(),
            )
        }

        LinearControllerType::RiccatiRecursionLQRInfinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon();
            Box::new(
                RiccatiRecursionLQR::new(sim, Box::new(cost.clone()), sim_time, dt, Some(options))
                    .unwrap(),
            )
        }
    };

    let u_traj = controller.solve(&initial_state).unwrap();
    controller.get_u_traj();
    let x_traj = controller.rollout(&initial_state).unwrap();

    let tol = 1e-3;
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

    match controller_type {
        LinearControllerType::QPLQRUlimits(lower, upper) => {
            let exceed_limits: Vec<_> = u_traj
                .iter()
                .filter(|u| u.to_vec()[0] < lower - tol || u.to_vec()[0] > upper + tol)
                .collect();
            assert!(exceed_limits.is_empty());
        }
        _ => (),
    }
}

#[test]
fn indirect_linear() {
    linear_controller_setup(LinearControllerType::IndirectShootingLQR);
}

#[test]
fn qp_lqr_linear() {
    linear_controller_setup(LinearControllerType::QPLQR);
}

#[test]
fn test_qp_lqr_linear_ulimits() {
    linear_controller_setup(LinearControllerType::QPLQRUlimits(-0.5, 0.5));
}

#[test]
fn test_ricatti_linear_infinite() {
    linear_controller_setup(LinearControllerType::RiccatiRecursionLQRInfinite);
}

#[test]
fn test_ricatti_linear_finite() {
    linear_controller_setup(LinearControllerType::RiccatiRecursionLQRFinite);
}
