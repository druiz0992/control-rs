use control_rs::controllers::qp_lqr::symbolic::QPLQRSymbolic;
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::riccati_lqr::symbolic::RiccatiRecursionSymbolic;
use control_rs::controllers::{Controller, ControllerOptions};
use control_rs::cost::generic::{GenericCost, GenericCostOptions};
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::quadrotor_2d::input::Quadrotor2DInput;
use control_rs::physics::models::quadrotor_2d::model::Quadrotor2D;
use control_rs::physics::models::quadrotor_2d::state::Quadrotor2DState;
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::State;
use nalgebra::{DMatrix, dvector};
use std::sync::Arc;

enum SymbolicControllerType {
    QPLQRSymbolic,
    QPLQRSymbolicLimits(f64, f64),
    RiccatiRecursionLQRFiniteSymbolic,
    RiccatiRecursionLQRInfiniteSymbolic,
}

type SymbolicContoller = Box<dyn Controller<BasicSim<Quadrotor2D, RK4Symbolic<Quadrotor2D>>>>;

fn symbolic_controller_setup(controller_type: SymbolicControllerType) {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    //let sim_time = 10.0;
    let sim_time = 1.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let input_hover = Quadrotor2DInput::new(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY);
    let state_hover = Quadrotor2DState::default();

    let initial_state = Quadrotor2DState::new(2.0, 4.0, 0.0, 0.0, 0.0, 0.0);
    let state_ref = Quadrotor2DState::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model, integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6);
    let qn_matrix = DMatrix::<f64>::identity(6, 6);
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let expected_trajectory: Vec<_> = (0..n_steps).map(|_| state_ref.clone()).collect();

    let options = GenericCostOptions::new().set_reference_state_trajectory(&expected_trajectory);
    let cost = GenericCost::new(
        q_matrix.clone(),
        qn_matrix.clone(),
        r_matrix.clone(),
        Some(options),
    )
    .unwrap();

    let general_options = ControllerOptions::<BasicSim<Quadrotor2D, RK4Symbolic<_>>>::default()
        .set_x_ref(&vec![state_ref.clone()])
        .set_u_operating(&input_hover)
        .set_x_operating(&state_hover);

    let mut controller: SymbolicContoller = match controller_type {
        SymbolicControllerType::QPLQRSymbolic => {
            let options = GenericCostOptions::new().set_linear_term(true);
            let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();
            let (controller, _) = QPLQRSymbolic::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                sim_time,
                dt,
                Some(general_options),
            )
            .unwrap();
            Box::new(controller)
        }
        SymbolicControllerType::QPLQRSymbolicLimits(lower, upper) => {
            let general_options = general_options.set_u_limits((lower, upper));
            let (controller, _) = QPLQRSymbolic::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                sim_time,
                dt,
                Some(general_options),
            )
            .unwrap();
            Box::new(controller)
        }
        SymbolicControllerType::RiccatiRecursionLQRFiniteSymbolic => {
            let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);

            let contoller = RiccatiRecursionSymbolic::new(
                sim,
                Box::new(cost.clone()),
                sim_time,
                dt,
                Some(options),
            )
            .unwrap();
            Box::new(contoller)
        }
        SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolic => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);

            let contoller = RiccatiRecursionSymbolic::new(
                sim,
                Box::new(cost.clone()),
                sim_time,
                dt,
                Some(options),
            )
            .unwrap();
            Box::new(contoller)
        }
    };

    controller.solve(&initial_state).unwrap();
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
        (u_traj.last().unwrap().to_vector() - dvector!(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY))
            .abs()
            .sum()
            < tol
    );

    match controller_type {
        SymbolicControllerType::QPLQRSymbolicLimits(lower, upper) => {
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
fn test_qp_lqr_symbolic() {
    symbolic_controller_setup(SymbolicControllerType::QPLQRSymbolic);
}

#[test]
fn test_qp_lqr_limits_symbolic() {
    symbolic_controller_setup(SymbolicControllerType::QPLQRSymbolicLimits(0.2*c::GRAVITY, 0.6*c::GRAVITY));
}

#[test]
fn test_ricatti_infinite_symbolic() {
    symbolic_controller_setup(SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolic);
}

#[test]
fn test_ricatti_finite_symbolic() {
    symbolic_controller_setup(SymbolicControllerType::RiccatiRecursionLQRFiniteSymbolic);
}
