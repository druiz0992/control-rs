use control_rs::controllers::qp_lqr::options::QPOptions;
use control_rs::controllers::qp_lqr::symbolic::QPLQRSymbolic;
use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::riccati_lqr::symbolic::RiccatiRecursionSymbolic;
use control_rs::controllers::{ConstraintTransform, Controller, ControllerOptions};
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
use osqp::Settings;
use std::sync::Arc;

enum SymbolicControllerType {
    QPLQRSymbolic,
    QPLQRSymbolicLimits(f64, f64),
    RiccatiRecursionLQRFiniteSymbolic,
    RiccatiRecursionLQRInfiniteSymbolic,
    RiccatiRecursionLQRInfiniteSymbolicLimits(f64, f64),
}

type QuadrotorSim = BasicSim<Quadrotor2D, RK4Symbolic<Quadrotor2D>>;
type SymbolicContoller = Box<dyn Controller<QuadrotorSim>>;

fn symbolic_controller_setup(controller_type: SymbolicControllerType, tol: f64) {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let input_hover = Quadrotor2DInput::new(0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY);
    let state_hover = Quadrotor2DState::default();

    let initial_state = Quadrotor2DState::new(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
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

    let general_options = ControllerOptions::<QuadrotorSim>::default()
        .set_x_ref(&[state_ref.clone()])
        .set_u_ref(&[input_hover.clone()])
        .set_u_operating(&input_hover)
        .set_x_operating(&state_hover)
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(sim_time)
        .unwrap();

    let mut controller: SymbolicContoller = match controller_type {
        SymbolicControllerType::QPLQRSymbolic => {
            let options = GenericCostOptions::new().set_linear_term(true);
            let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();

            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQRSymbolic::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                Some(qp_options),
            )
            .unwrap();
            Box::new(controller)
        }
        SymbolicControllerType::QPLQRSymbolicLimits(lower, upper) => {
            let options = GenericCostOptions::new().set_linear_term(true);
            let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, Some(options)).unwrap();
            let contraints = ConstraintTransform::new_uniform_bounds_input::<QuadrotorSim>((
                lower - input_hover.to_vec()[0],
                upper - input_hover.to_vec()[0],
            ));
            let general_options = general_options.set_u_limits(contraints);
            let osqp_settings = Settings::default().verbose(false);
            let qp_options = QPOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) = QPLQRSymbolic::new(
                sim,
                Box::new(cost.clone()),
                &initial_state,
                Some(qp_options),
            )
            .unwrap();
            Box::new(controller)
        }
        SymbolicControllerType::RiccatiRecursionLQRFiniteSymbolic => {
            let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);

            let contoller =
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap();
            Box::new(contoller)
        }
        SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolic => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);

            let contoller =
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap();
            Box::new(contoller)
        }
        SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolicLimits(lower, upper) => {
            let contraints =
                ConstraintTransform::new_uniform_bounds_input::<QuadrotorSim>((lower, upper));
            let general_options = general_options.set_u_limits(contraints);
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);

            let contoller =
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap();
            Box::new(contoller)
        }
    };

    let (x_traj, u_traj) = controller.solve(&initial_state).unwrap();

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
        SymbolicControllerType::QPLQRSymbolicLimits(lower, upper)
        | SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolicLimits(lower, upper) => {
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
    symbolic_controller_setup(SymbolicControllerType::QPLQRSymbolic, 1e-3);
}

#[test]
fn test_qp_lqr_limits_symbolic() {
    symbolic_controller_setup(
        SymbolicControllerType::QPLQRSymbolicLimits(0.2 * c::GRAVITY, 0.6 * c::GRAVITY),
        1e-3,
    );
}

#[test]
fn test_ricatti_infinite_symbolic() {
    symbolic_controller_setup(
        SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolic,
        1e-3,
    );
}

#[test]
fn test_ricatti_infinite_limits_symbolic() {
    symbolic_controller_setup(
        SymbolicControllerType::RiccatiRecursionLQRInfiniteSymbolicLimits(
            0.2 * c::GRAVITY,
            0.6 * c::GRAVITY,
        ),
        1e-3,
    );
}

#[test]
fn test_ricatti_finite_symbolic() {
    symbolic_controller_setup(
        SymbolicControllerType::RiccatiRecursionLQRFiniteSymbolic,
        1e-3,
    );
}
