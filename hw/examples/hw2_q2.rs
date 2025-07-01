use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::controllers::qp_lqr::{QPLQR, QPOptions};
use control_rs::controllers::qp_mpc::{ConvexMpc, ConvexMpcOptions};
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursion};
use control_rs::controllers::utils::clamp_input_vector;
use control_rs::controllers::{ConstraintTransform, Controller, ControllerOptions};
use control_rs::cost::GenericCost;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::{CartPole, CartPoleInput, CartPoleState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::{Discretizer, Dynamics, State};
use control_rs::physics::{Energy, constants as c};
use control_rs::plotter;
use control_rs::utils::Labelizable;
use hw::TrajOptions;
use nalgebra::{DMatrix, DVector};
use osqp::Settings;
use std::f64::consts::PI;
use std::io::{self, Write};
use std::sync::Arc;

type Sim = BasicSim<CartPole, RK4Symbolic<CartPole>>;

async fn generate_reference_traj(
    model: &CartPole,
    x_ic: &CartPoleState,
    dt: f64,
    registry: Arc<ExprRegistry>,
) -> (Vec<CartPoleState>, Vec<CartPoleInput>) {
    let desired_energy = Energy::new(0.0, 0.2 * c::GRAVITY * 0.5);
    let u_limits = (-14.0, 14.0);
    let constraints =
        ConstraintTransform::new_uniform_bounds_input::<Sim>((u_limits.0, u_limits.1));
    let mut traj_options = TrajOptions::new()
        .set_u_limits(constraints)
        .set_tf(21.0)
        .set_dt(dt);
    let (mut x_traj, mut u_traj) = energy_shaping_control(
        model,
        x_ic,
        &desired_energy,
        Arc::clone(&registry),
        &traj_options,
    );

    let q = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 0.01, 1.0, 0.01]));
    let r = DMatrix::identity(1, 1) * 1.0;

    let x_ref = CartPoleState::new(0.0, 0.0, 0.0, 0.0);
    let u_ref = CartPoleInput::new(0.0);

    traj_options = TrajOptions::new()
        .set_tf(10.0)
        .set_dt(dt)
        .set_finite_horizon(true)
        .set_q(q.clone())
        .set_qf(q.clone())
        .set_r(r.clone())
        .set_u_goal(vec![u_ref.clone()])
        .set_x_goal(vec![x_ref.clone()])
        .set_u_op(vec![u_ref.clone()])
        .set_x_op(vec![x_ref.clone()]);
    let mut controller = fhlqr(model, &traj_options, Arc::clone(&registry));
    let (x_traj2, u_traj2) = solve(
        &mut controller,
        model,
        x_traj.last().unwrap(),
        &traj_options,
    )
    .await;

    x_traj.extend(x_traj2);
    u_traj.extend(u_traj2);

    (x_traj, u_traj)
}

fn energy_shaping_control(
    model: &CartPole,
    x_ic: &CartPoleState,
    desired_energy: &Energy,
    registry: Arc<ExprRegistry>,
    traj_options: &TrajOptions<CartPoleState, CartPoleInput>,
) -> (Vec<CartPoleState>, Vec<CartPoleInput>) {
    let k = 20.0;
    let kp_x = 60.0;
    let kd_x = 100.0;
    let max_delta = 3.0;

    let tf = traj_options.tf;
    let dt = traj_options.dt;
    let n_steps = (tf / dt) as usize + 1;

    let mut x_traj = vec![];
    let mut u_traj: Vec<CartPoleInput> = vec![];
    x_traj.push(x_ic.clone());

    let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();

    for j in 0..n_steps - 1 {
        let current_energy = model.energy(&x_traj[j]).unwrap();
        let error = desired_energy.total() - current_energy.total();
        let [pos_x, v_x, theta, omega] = x_traj[j].extract(&["pos_x", "v_x", "theta", "omega"]);
        if theta.abs() < 0.1 && error < 0.1 {
            break;
        };
        let next_input_energy = DVector::from_column_slice(&[k * error * (omega * theta.cos())]);
        let cart_position_error = -pos_x;
        let cart_velocity = v_x;
        let next_input_cart =
            DVector::from_column_slice(&[kp_x * cart_position_error - kd_x * cart_velocity]);

        let mut u_candidate = next_input_cart + next_input_energy;

        if j > 0 {
            let u_prev = u_traj[j - 1].to_vector();
            let delta_u = &u_candidate - &u_prev;
            let delta_norm = delta_u.norm();
            if delta_norm > max_delta {
                let limited_delta = delta_u * (max_delta / delta_norm);
                u_candidate = u_prev + limited_delta;
            }
        }

        u_traj.push(CartPoleInput::from_vector(clamp_input_vector(
            u_candidate,
            traj_options.u_limits.as_ref(),
        )));
        x_traj.push(
            integrator
                .step(model, &x_traj[j], Some(&u_traj[j]), dt)
                .unwrap(),
        );
    }
    (x_traj, u_traj)
}
fn convex_trajopt(
    model: &CartPole,
    x_ic: &CartPoleState,
    registry: Arc<ExprRegistry>,
    traj_options: &TrajOptions<CartPoleState, CartPoleInput>,
) -> QPLQR<Sim> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let dt = traj_options.dt;

    // sim
    let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, Some(Arc::clone(&registry)));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    // cost
    let cost = GenericCost::<_, CartPoleInput>::new(q, qf, r, None).unwrap();

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
    let (controller, _) = QPLQR::new_symbolic(sim, Box::new(cost), x_ic, Some(qp_options)).unwrap();

    controller
}

fn convex_mpc(
    model: &CartPole,
    x_ic: &CartPoleState,
    registry: Arc<ExprRegistry>,
    traj_options: &TrajOptions<CartPoleState, CartPoleInput>,
) -> ConvexMpc<Sim, QPLQR<Sim>> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let dt = traj_options.dt;
    let mpc_horizon = traj_options.mpc_horizon;

    // sim
    let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, Some(Arc::clone(&registry)));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    // cost
    let cost = GenericCost::<_, CartPoleInput>::new(q, qf, r, None).unwrap();

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
    ConvexMpc::new_symbolic(sim, Box::new(cost.clone()), x_ic, Some(mpc_options)).unwrap()
}

fn fhlqr(
    model: &CartPole,
    traj_options: &TrajOptions<CartPoleState, CartPoleInput>,
    registry: Arc<ExprRegistry>,
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
    let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();
    let sim = BasicSim::new(model.clone(), integrator, Some(Arc::clone(&registry)));
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    // cost
    let q_matrix = q;
    let qn_matrix = qf;
    let r_matrix = r;
    let cost = GenericCost::<_, CartPoleInput>::new(q_matrix, qn_matrix, r_matrix, None).unwrap();

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
    RiccatiRecursion::new_symbolic(sim, Box::new(cost.clone()), Some(options)).unwrap()
}

async fn solve<T: Controller<Sim>>(
    controller: &mut T,
    model: &CartPole,
    x_ic: &CartPoleState,
    traj_options: &TrajOptions<CartPoleState, CartPoleInput>,
) -> (Vec<CartPoleState>, Vec<CartPoleInput>) {
    let dt = traj_options.dt;

    let (x_traj, u_traj) = controller.solve(x_ic).unwrap();

    if traj_options.plot_flag {
        let animation = Macroquad::new();

        animation
            .run_animation(model, &x_traj, (800.0, 600.0))
            .await
            .unwrap();
        let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();
        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
        plotter::plot_states(&times, &traj_options.x_goal, "/tmp/plot3.png").unwrap();

        plotter::display("/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();
        plotter::display("/tmp/plot3.png").unwrap();

        next("Ricatti Recursion");
    }

    (x_traj, u_traj)
}

fn input_trajectory_error(u1_traj: &[CartPoleInput], u2_traj: &[CartPoleInput]) -> f64 {
    u1_traj
        .iter()
        .zip(u2_traj)
        .map(|(u1, u2)| (u1.clone() - u2.clone()).to_vector().norm())
        .sum()
}
fn check_input_trajectory_error(
    u1_traj: &[CartPoleInput],
    u2_traj: &[CartPoleInput],
    tol: f64,
) -> f64 {
    let s = input_trajectory_error(u1_traj, u2_traj);
    assert!(s < tol);
    s
}

fn state_trajectory_error(x1_traj: &[CartPoleState], x2_traj: &[CartPoleState]) -> f64 {
    x1_traj
        .iter()
        .zip(x2_traj)
        .map(|(x1, x2)| (x1.clone() - x2.clone()).to_vector().norm())
        .sum()
}

fn check_state_trajectory_error(
    x1_traj: &[CartPoleState],
    x2_traj: &[CartPoleState],
    tol: f64,
) -> f64 {
    let s = state_trajectory_error(x1_traj, x2_traj) / x1_traj.len() as f64;
    assert!(s < tol);

    s
}
fn check_input_bounds(u_traj: &[CartPoleInput], u_limits: (f64, f64), tol: f64) {
    let (lower, upper) = u_limits;
    let exceed_limits: Vec<_> = u_traj
        .iter()
        .filter(|u| u.to_vec()[0] < lower - tol || u.to_vec()[0] > upper + tol)
        .collect();
    assert!(exceed_limits.is_empty());
}

fn next(msg: &str) {
    println!("{}. Press Enter to continue...", msg);
    let _ = io::stdout().flush();
    let _ = io::stdin().read_line(&mut String::new());
}

fn deg2rad(degrees: f64) -> f64 {
    degrees * 2.0 * PI / 360.0
}

#[macroquad::main("Ricatti Cart Pole")]
async fn main() {
    let registry = Arc::new(ExprRegistry::new());
    let model = CartPole::new(0.2, 1.0, 0.5, 0.0, 0.0, Some(&registry), true);
    let estimated_parameters = vec![0.16, 1.2, 0.55, 0.0, 0.0];

    let x_ref = CartPoleState::new(0.0, 0.0, 0.0, 0.0);
    let u_ref = CartPoleInput::new(0.0);

    // Part A - Drive cart pole to upwards equilibrium by setting a starting point a bit off the linearization. Also
    //  add estimated paramters to include simulation gap

    // initial condition (slightly off of our linearization point)
    let x_perturbation = CartPoleState::new(1.5, 0.3, deg2rad(-20.0), 0.0);
    let x_ic = x_ref.clone() + x_perturbation;

    let q = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 0.05, 1.0, 0.1]));
    let r = DMatrix::identity(1, 1) * 0.1;

    let traj_options = TrajOptions::new()
        .set_finite_horizon(true)
        .set_u_goal(vec![u_ref.clone()])
        .set_x_goal(vec![x_ref.clone()])
        .set_u_op(vec![u_ref.clone()])
        .set_x_op(vec![x_ref.clone()])
        .set_q(q.clone())
        .set_qf(q)
        .set_r(r)
        .set_estimated_parameters(estimated_parameters.clone());

    let mut controller = fhlqr(&model, &traj_options, Arc::clone(&registry));
    let (x_traj, u_traj) = solve(&mut controller, &model, &x_ic, &traj_options).await;
    check_input_trajectory_error(&[u_traj.last().unwrap().to_owned()], &[u_ref.clone()], 1e-1);
    check_state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()], 1e-1);

    // Part B - try a range of initial conditions to check which conditions are good enough for the linearization to work
    //  and drive the cart pole to the equilibrium position
    /*
    let thetas: Vec<f64> = (-10..=10).map(|x| x as f64 * 6.0).collect();
    let p_xs: Vec<f64> = (-10..=10).map(|x| x as f64 * 6.0).collect();

    let mut converging_conditions: Vec<(f64, f64)> = Vec::new();
    for theta in &thetas {
        for p_x in &p_xs {
            let x_ic = CartPoleState::new(*p_x, 0.0, deg2rad(*theta), 0.0);
            let (x_traj, _) = solve(&mut controller, &model, &x_ic, &traj_options).await;
            let state_err =
                state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()]);
            if state_err < 0.1 {
                converging_conditions.push((*theta, *p_x))
            }
        }
    }

    converging_conditions.iter().for_each(|(theta, x)| {
        assert!(*theta >= -48.0 && *theta <= 48.0);
        assert!(*x >= -6.0 && *x <= 6.0);
    });
    */

    // Part C - infinite horizon cost tuning.
    let x_perturbation = CartPoleState::new(0.5, 0.3, deg2rad(-10.0), 0.0);
    let x_ic = x_ref.clone() + x_perturbation;

    let q = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 0.01, 1.0, 0.01]));
    let r = DMatrix::identity(1, 1) * 1.0;

    let u_limits = (-3.0, 3.0);
    let constraints =
        ConstraintTransform::new_uniform_bounds_input::<Sim>((u_limits.0, u_limits.1));

    let traj_options = TrajOptions::new()
        .set_finite_horizon(false)
        .set_u_goal(vec![u_ref.clone()])
        .set_x_goal(vec![x_ref.clone()])
        .set_u_op(vec![u_ref.clone()])
        .set_x_op(vec![x_ref.clone()])
        .set_q(q.clone())
        .set_qf(q.clone())
        .set_r(r.clone())
        .set_estimated_parameters(estimated_parameters.clone())
        .set_u_limits(constraints);

    let mut controller = fhlqr(&model, &traj_options, Arc::clone(&registry));
    let (x_traj, u_traj) = solve(&mut controller, &model, &x_ic, &traj_options).await;
    check_state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()], 1e-1);
    check_input_bounds(&u_traj, u_limits, 1e-1);

    // Part D - TVLQR for trajectory tracking
    let x_ic = CartPoleState::new(0.0, 0.0, PI, 0.0);
    let model = CartPole::new(0.2, 1.0, 0.5, 0.0, 0.0, Some(&registry), true);

    let dt = 0.05;
    let (x_traj, u_traj) = generate_reference_traj(&model, &x_ic, dt, Arc::clone(&registry)).await;
    check_state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()], 1e-1);

    let tf = u_traj.len() as f64 * dt;
    let traj_options = TrajOptions::new()
        .set_tf(tf)
        .set_dt(dt)
        .set_u_goal(u_traj.clone())
        .set_x_goal(x_traj.clone())
        .set_u_op(u_traj.clone())
        .set_x_op(x_traj.clone())
        .set_estimated_parameters(estimated_parameters.clone());
    let mut controller = fhlqr(&model, &traj_options, Arc::clone(&registry));
    let (x_traj_final, _) = solve(
        &mut controller,
        &model,
        x_traj.first().unwrap(),
        &traj_options,
    )
    .await;
    check_state_trajectory_error(
        &[x_traj_final.last().unwrap().to_owned()],
        &[x_ref.clone()],
        1e-1,
    );

    // Part D: QP
    let traj_options = traj_options.set_plot_flag(true);
    let mut qplqr = convex_trajopt(&model, &x_ic, Arc::clone(&registry), &traj_options);
    let (_x_traj_qplr, _) = solve(&mut qplqr, &model, &x_ic, &traj_options).await;
    //check_state_trajectory_error(&x_traj_qplr, &x_traj, 1e-2);

    // Part E: MPC
    let traj_options = traj_options.set_noise(vec![0.01; 4]).set_mpc_horizon(1.0);
    let mut mpc = convex_mpc(&model, &x_ic, Arc::clone(&registry), &traj_options);
    let (_x_traj_mpc, _) = solve(&mut mpc, &model, &x_ic, &traj_options).await;
    //check_state_trajectory_error(&x_traj_mpc, &x_traj, 1e-2);
}
