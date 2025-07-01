use std::fs::File;
use std::io::{self, BufReader, Write};

use control_rs::controllers::qp_lqr::QPOptions;
use control_rs::controllers::qp_mpc::{ConvexMpc, ConvexMpcOptions};
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursion};
use control_rs::controllers::{ConstraintTransform, Controller, ControllerOptions, QPLQR};
use control_rs::cost::GenericCost;
use control_rs::physics::discretizer::{RK4, ZOH};
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::{Discretizer, State};
use control_rs::plotter;
use exercises::TrajOptions;
use nalgebra::{DMatrix, dmatrix};
use osqp::Settings;
use serde_json::from_reader;

const N: usize = 6;
const C: usize = 0;
const I: usize = 3;

const MU: f64 = 3.986_004_418e14; // [m^3/s^2]
const A: f64 = 6_971_100.0; // [m]

type Sim = BasicSim<LtiModel<N, C, I>, ZOH<LtiModel<N, C, I>>>;
type SpaceXDragon = LtiModel<N, C, I>;
type SpaceXDragonInput = LtiInput<I, C>;
type SpaceXDragonState = LtiState<N, C>;

fn load_traj<T: State>(path: &str) -> Result<Vec<T>, &'static str> {
    let file = File::open(path).map_err(|_| "Couldnt load file")?;

    let reader = BufReader::new(file);

    // Parse the JSON array of arrays
    let data: Vec<Vec<f64>> = from_reader(reader).map_err(|_| "Couldnt parse file")?;
    let traj: Vec<T> = data.iter().map(|v| T::from_slice(v)).collect();

    Ok(traj)
}

fn convex_trajopt(
    model: &SpaceXDragon,
    x_ic: &SpaceXDragonState,
    traj_options: &TrajOptions<SpaceXDragonState, SpaceXDragonInput>,
) -> QPLQR<Sim> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let dt = traj_options.dt;

    // sim
    let integrator = ZOH::new(model, dt).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, None);

    // cost
    let cost = GenericCost::<_, SpaceXDragonInput>::new(q, qf, r, None).unwrap();

    // setup QP controller
    let mut general_options = ControllerOptions::<Sim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(tf)
        .unwrap()
        .set_x_ref(&traj_options.x_goal.clone());
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
    let (controller, _) =
        QPLQR::new_linear(sim, Box::new(cost.clone()), x_ic, Some(qp_options)).unwrap();

    controller
}

fn convex_mpc(
    model: &SpaceXDragon,
    x_ic: &SpaceXDragonState,
    traj_options: &TrajOptions<SpaceXDragonState, SpaceXDragonInput>,
) -> ConvexMpc<Sim, QPLQR<Sim>> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let dt = traj_options.dt;
    let mpc_horizon = traj_options.mpc_horizon;

    // sim
    let integrator = ZOH::new(model, dt).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, None);

    // cost
    let cost = GenericCost::<_, SpaceXDragonInput>::new(q, qf, r, None).unwrap();

    // setup QP controller
    let mut general_options = ControllerOptions::<Sim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(tf)
        .unwrap()
        .set_x_ref(&traj_options.x_goal.clone());
    if let Some(noise) = &traj_options.noise_std {
        general_options = general_options.set_noise(noise.clone());
    }
    let osqp_settings = Settings::default()
        .verbose(false)
        .eps_abs(1e-8)
        .eps_rel(1e-8);
    let mpc_options = ConvexMpcOptions::default()
        .set_general(general_options)
        .set_osqp_settings(osqp_settings)
        .set_mpc_horizon(mpc_horizon);
    ConvexMpc::new_linear(sim, Box::new(cost.clone()), x_ic, Some(mpc_options)).unwrap()
}

fn fhlqr(
    model: &SpaceXDragon,
    traj_options: &TrajOptions<SpaceXDragonState, SpaceXDragonInput>,
) -> RiccatiRecursion<Sim> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let goal_state = traj_options.x_goal.clone();
    let goal_input = traj_options.u_goal.clone();
    let op_state = traj_options.x_op.clone();
    let op_input = traj_options.u_op.clone();
    let dt = traj_options.dt;

    // sim
    let integrator = ZOH::new(model, dt).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, None);

    // cost
    let q_matrix = q;
    let qn_matrix = qf;
    let r_matrix = r;
    let cost =
        GenericCost::<_, SpaceXDragonInput>::new(q_matrix, qn_matrix, r_matrix, None).unwrap();

    // setup QP controller
    let mut general_options = ControllerOptions::<Sim>::default()
        .set_dt(dt)
        .unwrap()
        .set_time_horizon(tf)
        .unwrap()
        .set_x_ref(&goal_state)
        .set_u_ref(&goal_input)
        .set_x_operating(&op_state)
        .set_u_operating(&op_input);

    if let Some(noise) = &traj_options.noise_std {
        general_options = general_options.set_noise(noise.clone());
    }
    if let Some(u_limits) = &traj_options.u_limits {
        general_options = general_options.set_u_limits(u_limits.clone());
    }
    if let Some(params) = &traj_options.estimated_parameters {
        general_options = general_options.set_estimated_params(params.clone());
    }

    let options = if traj_options.finite_horizon_flag {
        RiccatiLQROptions::enable_finite_horizon().set_general(general_options)
    } else {
        RiccatiLQROptions::enable_infinite_horizon().set_general(general_options)
    };
    RiccatiRecursion::new_linear(sim, Box::new(cost.clone()), Some(options)).unwrap()
}

fn solve<T: Controller<Sim>>(
    controller: &mut T,
    x_ic: &SpaceXDragonState,
    traj_options: &TrajOptions<SpaceXDragonState, SpaceXDragonInput>,
) -> (Vec<SpaceXDragonState>, Vec<SpaceXDragonInput>) {
    let dt = traj_options.dt;

    let (x_traj, u_traj) = controller.solve(x_ic).unwrap();

    if traj_options.plot_flag {
        let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();
        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
        plotter::plot_states(&times, &traj_options.x_goal, "/tmp/plot3.png").unwrap();

        plotter::display("/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();
        plotter::display("/tmp/plot3.png").unwrap();

        next("Controller");
    }

    (x_traj, u_traj)
}

fn state_trajectory_error(x1_traj: &[SpaceXDragonState], x2_traj: &[SpaceXDragonState]) -> f64 {
    x1_traj
        .iter()
        .zip(x2_traj)
        .map(|(x1, x2)| (x1.clone() - x2.clone()).to_vector().norm())
        .sum()
}

fn check_state_trajectory_error(
    x1_traj: &[SpaceXDragonState],
    x2_traj: &[SpaceXDragonState],
    tol: f64,
) -> f64 {
    let s = state_trajectory_error(x1_traj, x2_traj) / x1_traj.len() as f64;
    assert!(s / (x1_traj.len() as f64) < tol);

    s
}

fn next(msg: &str) {
    println!("{}. Press Enter to continue...", msg);
    let _ = io::stdout().flush();
    let _ = io::stdin().read_line(&mut String::new());
}

fn check_discrete_time_dynamics(model: &SpaceXDragon) {
    let dt = 0.1;
    let initial_state = SpaceXDragonState::new([1.0, 3.0, -0.3, 0.2, 0.4, -0.5]);
    let initial_input = SpaceXDragonInput::new([-0.1, 0.5, 0.3]);
    let integrator = ZOH::new(model, dt).unwrap();
    let integrator2 = RK4::new(model).unwrap();

    let new_state = integrator
        .step(model, &initial_state, Some(&initial_input), dt)
        .unwrap();
    let expected_state = integrator2
        .step(model, &initial_state, Some(&initial_input), dt)
        .unwrap();

    let tol = 1e-8;
    assert!((new_state - expected_state).to_vector().abs().sum() <= tol);
}
fn main() {
    let param_n: f64 = (MU / A.powf(3.0)).sqrt(); // [rad/s]
    let tf = 120.0;
    let dt = 1.0;
    let u_limits = (-0.4, 0.4);

    let state_matrix = dmatrix![0.0, 0.0, 0.0, 1.0, 0.0, 0.0; 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;0.0,0.0,0.0,0.0,0.0,1.0; 3.0 * param_n.powf(2.0), 0.0,0.0,0.0,2.0*param_n, 0.0; 0.0,0.0,0.0, -2.0* param_n, 0.0,0.0; 0.0,0.0,-param_n.powf(2.0),0.0,0.0,0.0 ];
    let control_matrix =
        dmatrix![0.0,0.0,0.0;0.0,0.0,0.0;0.0,0.0,0.0;1.0,0.0,0.0;0.0,1.0,0.0;0.0,0.0,1.0];
    let model = SpaceXDragon::new(state_matrix, control_matrix).unwrap();

    // Part A - Dyscretize dynamics
    check_discrete_time_dynamics(&model);

    // Part B - LQR
    let x_ic = SpaceXDragonState::new([-2.0, -4.0, 2.0, 0.0, 0.0, 0.0]);

    let q = DMatrix::<f64>::identity(6, 6);
    let r = DMatrix::<f64>::identity(3, 3);
    let qf = DMatrix::<f64>::identity(6, 6) * 10.0;

    let x_ref = load_traj::<SpaceXDragonState>("./hw/examples/data/spacex_x_traj1.dat").unwrap();

    let constraints = ConstraintTransform::new_uniform_bounds_input::<Sim>(u_limits);
    let traj_options = TrajOptions::new()
        .set_plot_flag(true)
        .set_tf(tf)
        .set_dt(dt)
        .set_u_limits(constraints)
        .set_x_goal(x_ref.clone())
        .set_q(q)
        .set_qf(qf)
        .set_r(r)
        .set_finite_horizon(true);
    let mut ricatti = fhlqr(&model, &traj_options);
    let (x_traj, _) = solve(&mut ricatti, &x_ic, &traj_options);
    check_state_trajectory_error(&x_traj, &x_ref, 1e-2);

    // Part C - Convex trajectory optimization
    let mut qplqr = convex_trajopt(&model, &x_ic, &traj_options);
    let (x_traj, _) = solve(&mut qplqr, &x_ic, &traj_options);
    check_state_trajectory_error(&x_traj, &x_ref, 1e-2);

    // Part D: MPC
    let traj_options = traj_options.set_noise(vec![0.01; N]).set_mpc_horizon(20.0);
    let mut mpc = convex_mpc(&model, &x_ic, &traj_options);
    let (x_traj, _) = solve(&mut mpc, &x_ic, &traj_options);
    check_state_trajectory_error(&x_traj, &x_ref, 1e-2);
}
