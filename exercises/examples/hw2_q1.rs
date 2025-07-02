use std::io::{self, Write};

use control_rs::controllers::qp_lqr::QPOptions;
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursion};
use control_rs::controllers::{Controller, ControllerOptions, QPLQR};
use control_rs::cost::GenericCost;
use control_rs::physics::discretizer::ZOH;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::{Discretizer, State};
use control_rs::plotter;
use control_rs::utils::noise::NoiseSources;
use exercises::TrajOptions;
use nalgebra::{DMatrix, dmatrix};
use osqp::Settings;

type DoubleIntegrator = LtiModel<4, 0, 2>;
type DoubleIntegratorState = LtiState<4, 0>;
type DoubleIntegratorInput = LtiInput<2, 0>;
type DoubleIntegratorSim = BasicSim<DoubleIntegrator, ZOH<DoubleIntegrator>>;

fn double_integrator() -> DoubleIntegrator {
    let state_matrix =
        dmatrix![0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0; 0.0,0.0,0.0,0.0; 0.0,0.0,0.0,0.0];
    let control_matrix = dmatrix![0.0,0.0;0.0,0.0;1.0,0.0; 0.0, 1.0];
    LtiModel::<4, 0, 2>::new(state_matrix, control_matrix).unwrap()
}

fn check_discrete_time_dynamics(model: &DoubleIntegrator) {
    let dt = 0.1;
    let initial_state = DoubleIntegratorState::new([1.0, 2.0, 3.0, 4.0]);
    let initial_input = DoubleIntegratorInput::new([-1.0, -3.0]);
    let integrator = ZOH::new(model, dt).unwrap();

    let new_state = integrator
        .step(model, &initial_state, Some(&initial_input), dt)
        .unwrap();
    let expected_state = DoubleIntegratorState::new([1.295, 2.385, 2.9, 3.7]);

    let tol = 1e-8;
    assert!((new_state - expected_state).to_vector().abs().sum() <= tol,);
}

fn convex_trajopt(
    model: &DoubleIntegrator,
    x_ic: &DoubleIntegratorState,
    traj_options: &TrajOptions<DoubleIntegratorState, DoubleIntegratorInput>,
) -> (Vec<DoubleIntegratorState>, Vec<DoubleIntegratorInput>) {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();

    // setup sim params
    let dt = 0.1;

    // sim
    let integrator = ZOH::new(model, dt).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    // cost
    let q_matrix = DMatrix::<f64>::identity(4, 4);
    let r_matrix = DMatrix::<f64>::identity(2, 2);
    let cost = GenericCost::<_, DoubleIntegratorInput>::new(q_matrix, qf, r_matrix, None).unwrap();

    // setup QP controller
    let mut general_options = ControllerOptions::<DoubleIntegratorSim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(tf)
        .unwrap()
        .set_x_ref(&traj_options.x_goal);
    if let Some(noise) = &traj_options.noise_std {
        general_options = general_options.set_noise(noise.clone());
    }
    let osqp_settings = Settings::default()
        .verbose(false)
        .eps_abs(1e-8)
        .eps_rel(1e-8);
    let qp_options = QPOptions::default()
        .set_general(general_options)
        .set_osqp_settings(osqp_settings);
    let (mut controller, _) =
        QPLQR::new_linear(sim, Box::new(cost.clone()), x_ic, Some(qp_options)).unwrap();

    let (x_traj, u_traj) = controller.solve(x_ic).unwrap();

    if traj_options.plot_flag {
        let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();
        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();

        plotter::display("/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();

        next("Convex Optimization");
    }

    (x_traj, u_traj)
}

fn fhlqr(
    model: &DoubleIntegrator,
    x_ic: &DoubleIntegratorState,
    traj_options: &TrajOptions<DoubleIntegratorState, DoubleIntegratorInput>,
) -> (Vec<DoubleIntegratorState>, Vec<DoubleIntegratorInput>) {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();

    // setup sim params
    let dt = 0.1;

    // sim
    let integrator = ZOH::new(model, dt).unwrap();
    let sim = BasicSim::new(model.clone(), integrator);

    // cost
    let q_matrix = DMatrix::<f64>::identity(4, 4);
    let r_matrix = DMatrix::<f64>::identity(2, 2);
    let cost = GenericCost::<_, DoubleIntegratorInput>::new(q_matrix, qf, r_matrix, None).unwrap();

    // setup QP controller
    let mut general_options = ControllerOptions::<DoubleIntegratorSim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(tf)
        .unwrap()
        .set_x_ref(&traj_options.x_goal);

    if let Some(noise) = &traj_options.noise_std {
        general_options = general_options.set_noise(noise.clone());
    }

    let options = RiccatiLQROptions::enable_finite_horizon().set_general(general_options);
    let mut controller =
        RiccatiRecursion::new_linear(sim, Box::new(cost.clone()), Some(options)).unwrap();

    let (x_traj, u_traj) = controller.solve(x_ic).unwrap();

    if traj_options.plot_flag {
        let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();
        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();

        plotter::display("/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();

        next("Ricatti Recursion");
    }

    (x_traj, u_traj)
}

fn sample_x_ic(
    model: &DoubleIntegrator,
    n_iter: usize,
    traj_options: &TrajOptions<DoubleIntegratorState, DoubleIntegratorInput>,
) {
    let noise_sources = NoiseSources::from_stats(vec![5.0, 5.0, 1.0, 1.0]).unwrap();
    for _ in 0..n_iter {
        let x = noise_sources
            .add_noise(DoubleIntegratorState::default().to_vector())
            .unwrap();
        let x_ic = DoubleIntegratorState::from_slice(x.as_slice());
        let (x_cvx_traj, u_cvx_traj) = convex_trajopt(model, &x_ic, traj_options);
        let (x_fhlqr_traj, u_fhlqr_traj) = fhlqr(model, &x_ic, traj_options);

        check_input_trajectory_error(&u_cvx_traj[..], &u_fhlqr_traj[..]);
        check_state_trajectory_error(&x_cvx_traj[..], &x_fhlqr_traj[..]);
    }
}
fn check_input_trajectory_error(
    u1_traj: &[DoubleIntegratorInput],
    u2_traj: &[DoubleIntegratorInput],
) -> f64 {
    let s: f64 = u1_traj
        .iter()
        .zip(u2_traj)
        .map(|(u1, u2)| (u1.clone() - u2.clone()).to_vector().abs().sum())
        .sum();

    assert!(s < 1e-6);

    s
}

fn check_state_trajectory_error(
    x1_traj: &[DoubleIntegratorState],
    x2_traj: &[DoubleIntegratorState],
) -> f64 {
    let s: f64 = x1_traj
        .iter()
        .zip(x2_traj)
        .map(|(x1, x2)| (x1.clone() - x2.clone()).to_vector().abs().sum())
        .sum();

    assert!(s < 1e-6);

    s
}

fn next(msg: &str) {
    println!("{}. Press Enter to continue...", msg);
    let _ = io::stdout().flush();
    let _ = io::stdin().read_line(&mut String::new());
}

fn main() {
    let traj_options = TrajOptions::new();

    // Part A - check dynamics
    let model = double_integrator();
    check_discrete_time_dynamics(&model);

    // Part B - use convex optimization LQR to come up with a trajectory
    // initial conditions
    let x_ic = DoubleIntegratorState::new([5.0, 7.0, 2.0, -1.4]);
    let (x1_traj, u1_traj) = convex_trajopt(&model, &x_ic, &traj_options);

    // Compute new trajectory starting in a mid point of the previous one.
    // According to Bellman optimiality principle, both subtrajectories from L to N-1
    // should match
    let l = 18;
    let x2_ic = &x1_traj[l];
    let tf = traj_options.tf - 1.8;
    let new_traj_options = TrajOptions::new().set_tf(tf);
    let (x2_traj, u2_traj) = convex_trajopt(&model, x2_ic, &new_traj_options);

    let _u_err = check_input_trajectory_error(&u1_traj[l..], &u2_traj[..]);
    let _x_err = check_state_trajectory_error(&x1_traj[l..], &x2_traj[..]);

    // Part C - Convex optimization and Ricatti LQR should produce same
    // trajectory if initial conditions match
    let x_ic = DoubleIntegratorState::new([5.0, 7.0, 2.0, -1.4]);
    let (x3_traj, u3_traj) = fhlqr(&model, &x_ic, &traj_options);

    let _u_err = check_input_trajectory_error(&u1_traj[..], &u3_traj[..]);
    let _x_err = check_state_trajectory_error(&x1_traj[..], &x3_traj[..]);

    // sample different initial conditions and check that both Convex optimization and Ricatti give
    // same results
    sample_x_ic(&model, 20, &traj_options);

    //  Part D - Add noise and compare Convex optimization and  LQR.
    let qf = DMatrix::<f64>::identity(4, 4) * 10.0;
    let traj_options = TrajOptions::new()
        .set_noise(vec![0.1, 0.1, 1.0, 1.0])
        .set_qf(qf.clone())
        .set_tf(7.0);

    convex_trajopt(&model, &x_ic, &traj_options);
    fhlqr(&model, &x_ic, &traj_options);

    // Add goal state
    let x_goal = DoubleIntegratorState::new([-3.5, -3.5, 0.0, 0.0]);
    let traj_options = TrajOptions::new()
        .set_qf(qf)
        .set_tf(7.0)
        .set_x_goal(vec![x_goal]);

    let x_ic = DoubleIntegratorState::new([5.0, 7.0, 2.0, -1.4]);
    convex_trajopt(&model, &x_ic, &traj_options);
    fhlqr(&model, &x_ic, &traj_options);
}
