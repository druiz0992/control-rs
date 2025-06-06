use control_rs::controllers::qp_lqr::{QPLQRSymbolic, QPOptions};
use control_rs::controllers::qp_mpc::{ConvexMpcOptions, ConvexMpcSymbolic};
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursionSymbolic};
use control_rs::controllers::{ConstraintTransform, Controller, ControllerOptions};
use control_rs::cost::generic::GenericCost;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::quadrotor_2d::{Quadrotor2D, Quadrotor2DInput, Quadrotor2DState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::State;
use control_rs::utils::Labelizable;
use nalgebra::{DMatrix, dvector};
use osqp::Settings;
use std::sync::Arc;

enum ControllerType {
    QpLqr,
    QpLqrUlimits(f64, f64),
    RiccatiRecursionLQRFinite,
    RiccatiRecursionLQRInfinite,
    RiccatiRecursionLQRFiniteULimitsAndNoise(f64, f64, f64, f64),
    RiccatiRecursionLQRInfiniteULimitsAndNoise(f64, f64, f64, f64),
    Mpc,
    MpcULimitsAndNoise(f64, f64, f64, f64),
    MpcUXLimitsAndNoise(f64, f64, f64, f64, usize, f64, f64),
}

type Sim<M> = BasicSim<M, RK4Symbolic<M>>;
type SymbolicController<M> = Box<dyn Controller<Sim<M>>>;

fn symbolic_controller_setup(controller_type: ControllerType) {
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
    let sim = BasicSim::new(model.clone(), integrator, Some(Arc::clone(&registry)));

    let q_matrix = DMatrix::<f64>::identity(6, 6) * 1.0;
    let qn_matrix = DMatrix::<f64>::identity(6, 6) * 1.0;
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let mut expected_trajectory: Vec<_> = (0..n_steps + 1)
        .map(|_| Quadrotor2DState::default())
        .collect();
    expected_trajectory[n_steps] = state_ref.clone();

    let cost =
        GenericCost::new(q_matrix.clone(), qn_matrix.clone(), r_matrix.clone(), None).unwrap();

    let general_options = ControllerOptions::<Sim<Quadrotor2D>>::default()
        .set_x_ref(&[state_ref.clone()])
        .set_u_ref(&[input_hover.clone()])
        .set_x_operating(&state_hover)
        .set_u_operating(&input_hover)
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
                QPLQRSymbolic::new(sim, Box::new(cost.clone()), &state_0, Some(qp_options))
                    .unwrap();
            Box::new(controller)
        }
        ControllerType::QpLqrUlimits(lower, upper) => {
            let [hover] = input_hover.extract(&["u1"]);
            let constraints = ConstraintTransform::new_uniform_bounds_input::<Sim<Quadrotor2D>>((
                lower - hover,
                upper - hover,
            ));
            let general_options = general_options.set_u_limits(constraints);
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let qp_options = QPOptions::<Sim<Quadrotor2D>>::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            let (controller, _) =
                QPLQRSymbolic::new(sim, Box::new(cost.clone()), &state_0, Some(qp_options))
                    .unwrap();
            Box::new(controller)
        }
        ControllerType::RiccatiRecursionLQRFinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }

        ControllerType::RiccatiRecursionLQRInfinite => {
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        ControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(lower, upper, std_0, std_n) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<Sim<Quadrotor2D>>((lower, upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap(),
            )
        }
        ControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(lower, upper, std_0, std_n) => {
            let constraints =
                ConstraintTransform::new_uniform_bounds_input::<Sim<Quadrotor2D>>((lower, upper));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let options = RiccatiLQROptions::enable_infinite_horizon().set_general(general_options);
            Box::new(
                RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap(),
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
                ConvexMpcSymbolic::new(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
        }
        ControllerType::MpcULimitsAndNoise(lower, upper, std_0, std_n) => {
            let [hover] = input_hover.extract(&["u1"]);
            let constraints = ConstraintTransform::new_uniform_bounds_input::<Sim<Quadrotor2D>>((
                lower - hover,
                upper - hover,
            ));
            let general_options = general_options
                .set_u_limits(constraints)
                .set_noise((std_0, std_n));
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpcSymbolic::new(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
        }
        ControllerType::MpcUXLimitsAndNoise(
            lower_u,
            upper_u,
            lower_x,
            upper_x,
            idx,
            std_0,
            std_n,
        ) => {
            let [hover] = input_hover.extract(&["u1"]);
            let input_constraints = ConstraintTransform::new_uniform_bounds_input::<Sim<Quadrotor2D>>(
                (lower_u - hover, upper_u - hover),
            );

            let state_constraints =
                ConstraintTransform::new_single_bound_state::<Sim<Quadrotor2D>>(
                    (lower_x, upper_x),
                    idx,
                )
                .unwrap();
            let general_options = general_options
                .set_u_limits(input_constraints)
                .set_x_limits(state_constraints)
                .set_noise((std_0, std_n));
            let osqp_settings = Settings::default()
                .verbose(false)
                .eps_abs(1e-8)
                .eps_rel(1e-8);
            let options = ConvexMpcOptions::default()
                .set_general(general_options)
                .set_osqp_settings(osqp_settings);
            Box::new(
                ConvexMpcSymbolic::new(sim, Box::new(cost.clone()), &state_0, Some(options))
                    .unwrap(),
            )
        }
    };

    let (x_traj, u_traj) = controller.solve(&state_0).unwrap();

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
        ControllerType::QpLqrUlimits(lower, upper)
        | ControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(lower, upper, _, _)
        | ControllerType::MpcULimitsAndNoise(lower, upper, _, _)
        | ControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(lower, upper, _, _) => {
            let exceed_limits: Vec<_> = u_traj
                .iter()
                .filter(|u| u.to_vec()[0] < lower - tol || u.to_vec()[0] > upper + tol)
                .collect();
            assert!(exceed_limits.is_empty());
        }
        ControllerType::MpcUXLimitsAndNoise(_, _, lower, upper, idx, _, _) => {
            let exceed_limits: Vec<_> = x_traj
                .iter()
                .filter(|x| x.to_vec()[idx] < lower - tol || x.to_vec()[idx] > upper + tol)
                .collect();
            assert!(exceed_limits.is_empty());
        }
        _ => (),
    }
}

#[test]
#[should_panic]
fn test_qp_lqr_symbolic() {
    symbolic_controller_setup(ControllerType::QpLqr);
}

#[test]
#[should_panic]
fn test_qp_lqr_limits_symbolic() {
    symbolic_controller_setup(ControllerType::QpLqrUlimits(
        0.2 * c::GRAVITY,
        0.6 * c::GRAVITY,
    ));
}

#[test]
fn test_ricatti_finite_symbolic() {
    symbolic_controller_setup(ControllerType::RiccatiRecursionLQRFinite);
}

#[test]
fn test_ricatti_infinite_symbolic() {
    symbolic_controller_setup(ControllerType::RiccatiRecursionLQRInfinite);
}

#[test]
fn test_ricatti_finite_limits_symbolic() {
    symbolic_controller_setup(ControllerType::RiccatiRecursionLQRFiniteULimitsAndNoise(
        0.2 * c::GRAVITY,
        0.6 * c::GRAVITY,
        0.0,
        0.0,
    ));
}

#[test]
fn test_ricatti_infinite_limits_symbolic() {
    symbolic_controller_setup(ControllerType::RiccatiRecursionLQRInfiniteULimitsAndNoise(
        0.2 * c::GRAVITY,
        0.6 * c::GRAVITY,
        0.0,
        0.0,
    ));
}

#[test]
fn test_mpc_symbolic() {
    symbolic_controller_setup(ControllerType::Mpc);
}

#[test]
fn test_mpc_limits_symbolic() {
    symbolic_controller_setup(ControllerType::MpcULimitsAndNoise(
        0.2 * c::GRAVITY,
        0.6 * c::GRAVITY,
        0.0,
        0.0,
    ));
}

#[test]
fn test_mpc_uxlimits_symbolic() {
    symbolic_controller_setup(ControllerType::MpcUXLimitsAndNoise(
        0.2 * c::GRAVITY,
        0.6 * c::GRAVITY,
        -0.2,
        0.2,
        2,
        0.0,
        0.0,
    ));
}
