use std::f64::consts::PI;
use std::fs;
use std::io::{self, Write};
use std::sync::Arc;

use control_rs::animation::Animation;
use control_rs::animation::macroquad::Macroquad;
use control_rs::controllers::riccati_lqr::{RiccatiLQROptions, RiccatiRecursionSymbolic};
use control_rs::controllers::{ConstraintTransform, Controller, ControllerOptions};
use control_rs::cost::GenericCost;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::RK4Symbolic;
use control_rs::physics::models::{CartPole, CartPoleInput, CartPoleState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::{Discretizer, Dynamics, State};
use control_rs::plotter;
use control_rs::utils::NoiseSource;
use nalgebra::{DMatrix, DVector};

type Sim = BasicSim<CartPole, RK4Symbolic<CartPole>>;

const DEFAULT_QF_FACTOR: f64 = 5.0;
const DEFAULT_Q_FACTOR: f64 = 1.0;
const DEFAULT_R_FACTOR: f64 = 1.0;
const DEFAULT_TF: f64 = 5.0;

#[derive(Clone, Debug)]
struct TrajOptions {
    qf: DMatrix<f64>,
    q: DMatrix<f64>,
    r: DMatrix<f64>,
    noise_std: Option<(f64, f64)>,
    tf: f64,
    plot_flag: bool,
    x_goal: CartPoleState,
    u_goal: CartPoleInput,
    x_op: CartPoleState,
    u_op: CartPoleInput,
    finite_horizon_flag: bool,
    estimated_parameters: Option<Vec<f64>>,
    u_imits: Option<ConstraintTransform>,
}

impl TrajOptions {
    fn new() -> Self {
        TrajOptions::default()
    }
    fn set_qf(self, qf: DMatrix<f64>) -> Self {
        let mut new = self;
        new.qf = qf;
        new
    }
    fn set_r(self, r: DMatrix<f64>) -> Self {
        let mut new = self;
        new.r = r;
        new
    }
    fn set_q(self, q: DMatrix<f64>) -> Self {
        let mut new = self;
        new.q = q;
        new
    }
    fn set_noise(self, noise_sources: Option<(f64, f64)>) -> Self {
        let mut new = self;
        new.noise_std = noise_sources;
        new
    }
    fn set_tf(self, tf: f64) -> Self {
        let mut new = self;
        new.tf = tf;
        new
    }
    fn set_plot_flag(self, flag: bool) -> Self {
        let mut new = self;
        new.plot_flag = flag;
        new
    }
    fn set_x_goal(self, x_goal: CartPoleState) -> Self {
        let mut new = self;
        new.x_goal = x_goal;
        new
    }
    fn set_u_goal(self, u_goal: CartPoleInput) -> Self {
        let mut new = self;
        new.u_goal = u_goal;
        new
    }
    fn set_x_op(self, x_op: CartPoleState) -> Self {
        let mut new = self;
        new.x_op = x_op;
        new
    }
    fn set_u_op(self, u_op: CartPoleInput) -> Self {
        let mut new = self;
        new.u_op = u_op;
        new
    }
    fn set_finite_horizon(self, flag: bool) -> Self {
        let mut new = self;
        new.finite_horizon_flag = flag;
        new
    }
    fn set_estimated_parameters(self, params: Vec<f64>) -> Self {
        let mut new = self;
        new.estimated_parameters = Some(params);
        new
    }
    fn set_u_limits(self, constraints: ConstraintTransform) -> Self {
        let mut new = self;
        new.u_imits = Some(constraints);
        new
    }
}

impl Default for TrajOptions {
    fn default() -> Self {
        let state_dims = CartPoleState::dim_q();
        let input_dims = CartPoleInput::dim_q();
        Self {
            tf: DEFAULT_TF,
            qf: DMatrix::identity(state_dims, state_dims) * DEFAULT_QF_FACTOR,
            q: DMatrix::identity(state_dims, state_dims) * DEFAULT_Q_FACTOR,
            r: DMatrix::identity(input_dims, input_dims) * DEFAULT_R_FACTOR,
            noise_std: None,
            plot_flag: false,
            x_goal: CartPoleState::default(),
            u_goal: CartPoleInput::default(),
            x_op: CartPoleState::default(),
            u_op: CartPoleInput::default(),
            finite_horizon_flag: true,
            estimated_parameters: None,
            u_imits: None,
        }
    }
}

fn load_u_traj(path: &str) -> Result<Vec<CartPoleInput>, &'static str> {
    let contents = fs::read_to_string(path).map_err(|_| "Couldnt load file")?;

    let data = contents
        .split(',')
        .map(str::trim) // remove surrounding whitespace
        .filter(|s| !s.is_empty())
        .map(|s| CartPoleInput::from_vec(vec![s.parse::<f64>().unwrap()]))
        .collect::<Vec<CartPoleInput>>();

    Ok(data)
}

fn generate_u_traj(dt: f64, total_time: f64, freq: f64, amp: f64) -> Vec<CartPoleInput> {
    let steps = (total_time / dt) as usize;

    // swing back and forth with increasing energy
    let u_traj: Vec<CartPoleInput> = (0..steps)
        .map(|i| {
            let t = i as f64 * dt;
            let amp = if t <= 2.0 { amp } else { 0.0 };
            CartPoleInput::from_vec(vec![amp * (2.0 * PI * freq * t).sin()])
        })
        .collect();
    u_traj
}

fn rollout(
    model: &CartPole,
    x_ic: &CartPoleState,
    u_traj: &[CartPoleInput],
    registry: Arc<ExprRegistry>,
) -> Vec<CartPoleState> {
    let dt = 0.05;
    let integrator = RK4Symbolic::new(model, Arc::clone(&registry)).unwrap();
    let mut x_traj = vec![x_ic.clone(); u_traj.len() + 1];
    for (k, u) in u_traj.iter().enumerate() {
        x_traj[k + 1] = integrator.step(model, &x_traj[k], Some(u), dt).unwrap();
    }
    x_traj
}
async fn fhlqr(
    model: &CartPole,
    traj_options: &TrajOptions,
    registry: Arc<ExprRegistry>,
) -> RiccatiRecursionSymbolic<Sim> {
    let tf = traj_options.tf;
    let qf = traj_options.qf.clone();
    let q = traj_options.q.clone();
    let r = traj_options.r.clone();
    let goal_state = traj_options.x_goal.clone();
    let goal_input = traj_options.u_goal.clone();
    let op_state = traj_options.x_op.clone();
    let op_input = traj_options.u_op.clone();

    // setup sim params
    let dt = 0.1;

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
        .set_x_ref(&[goal_state])
        .set_u_ref(&[goal_input])
        .set_x_operating(&op_state)
        .set_u_operating(&op_input);

    if let Some(noise) = traj_options.noise_std {
        general_options = general_options.set_noise(noise);
    }
    if let Some(params) = &traj_options.estimated_parameters {
        general_options = general_options.set_estimated_params(params.clone());
    }

    let options = if traj_options.finite_horizon_flag {
        RiccatiLQROptions::enable_finite_horizon().set_general(general_options)
    } else {
        RiccatiLQROptions::enable_infinite_horizon().set_general(general_options)
    };
    RiccatiRecursionSymbolic::new(sim, Box::new(cost.clone()), Some(options)).unwrap()
}

async fn solve(
    controller: &mut RiccatiRecursionSymbolic<Sim>,
    model: &CartPole,
    x_ic: &CartPoleState,
    traj_options: &TrajOptions,
) -> (Vec<CartPoleState>, Vec<CartPoleInput>) {
    let dt = 0.1;
    let (x_traj, u_traj) = controller.solve(&x_ic).unwrap();

    if traj_options.plot_flag {
        let animation = Macroquad::new();

        animation
            .run_animation(model, &x_traj, (800.0, 600.0))
            .await
            .unwrap();
        let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * dt).collect();
        plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
        plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();

        plotter::display("/tmp/plot1.png").unwrap();
        plotter::display("/tmp/plot2.png").unwrap();

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
    let s = state_trajectory_error(x1_traj, x2_traj);
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

fn add_noise<S: State>(noise_sources: (&NoiseSource, &NoiseSource, usize), state: S) -> S {
    let (noise_source_1, noise_source_2, idx) = noise_sources;
    let s = S::default().to_vector();

    let s1 = noise_source_1.add_noise(DVector::from_vec(s.as_slice()[..idx].to_vec()));
    let s2 = noise_source_2.add_noise(DVector::from_vec(s.as_slice()[idx..].to_vec()));
    let mut x_ic = s1.as_slice().to_vec();
    x_ic.extend_from_slice(s2.as_slice());

    let noise = S::from_vec(x_ic);
    noise + state
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
    let model = CartPole::new(0.16, 1.2, 0.1, 0.1, 0.55, Some(&registry));
    let estimated_parameters = vec![0.2, 1.0, 0.0, 0.0, 0.50];

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
        .set_u_goal(u_ref.clone())
        .set_x_goal(x_ref.clone())
        .set_u_op(u_ref.clone())
        .set_x_op(x_ref.clone())
        .set_q(q.clone())
        .set_qf(q)
        .set_r(r)
        .set_estimated_parameters(estimated_parameters.clone());
    /* *
        let mut controller = fhlqr(&model, &traj_options, Arc::clone(&registry)).await;
        let (x_traj, u_traj) = solve(&mut controller, &model, &x_ic, &traj_options).await;
        check_input_trajectory_error(&[u_traj.last().unwrap().to_owned()], &[u_ref.clone()], 1e-1);
        check_state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()], 1e-1);

        // Part B - try a range of initial conditions to check which conditions are good enough for the linearization to work
        //  and drive the cart pole to the equilibrium position
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

        // Part C - infinite horizon cost tuning.
        let x_perturbation = CartPoleState::new(0.5, 0.3, deg2rad(-10.0), 0.0);
        let x_ic = x_ref.clone() + x_perturbation;

        let q = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 0.01, 1.0, 0.01]));
        let r = DMatrix::identity(1, 1) * 1.0;

        let u_limits = (-3.0, 3.0);
        let constraints =
            ConstraintTransform::new_uniform_bounds_input::<Sim>((u_limits.0, u_limits.1));

        let traj_options = TrajOptions::new()
            .set_finite_horizon(true)
            .set_u_goal(u_ref.clone())
            .set_x_goal(x_ref.clone())
            .set_u_op(u_ref.clone())
            .set_x_op(x_ref.clone())
            .set_q(q.clone())
            .set_qf(q)
            .set_r(r)
            .set_estimated_parameters(estimated_parameters.clone())
            .set_u_limits(constraints)
            .set_plot_flag(true);

        let mut controller = fhlqr(&model, &traj_options, Arc::clone(&registry)).await;
        let (x_traj, u_traj) = solve(&mut controller, &model, &x_ic, &traj_options).await;
        check_state_trajectory_error(&[x_traj.last().unwrap().to_owned()], &[x_ref.clone()], 1e-1);
        check_input_bounds(&u_traj, u_limits, 1e-1);
    */
    // Part D - TVLQR for trajectory tracking
    let x_ic = CartPoleState::new(0.0, 0.0, PI, 0.0);
    let model = CartPole::new(0.2, 1.0, 0.0, 0.0, 0.50, Some(&registry));
    //let u_traj = load_u_traj("./hw/examples/data/cartpole_u_traj1.dat").unwrap();
    let u_traj = generate_u_traj(0.05, 4.0, 0.7, 10.0);
    let x_traj = rollout(&model, &x_ic, &u_traj, registry);

    let times: Vec<_> = (0..u_traj.len()).map(|i| i as f64 * 0.05).collect();
    plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
    plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
    plotter::display("/tmp/plot1.png").unwrap();

    dbg!(&x_traj, &u_traj);
}
