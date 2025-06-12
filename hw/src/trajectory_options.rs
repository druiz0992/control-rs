use control_rs::{controllers::ConstraintTransform, physics::traits::State};
use nalgebra::DMatrix;

const DEFAULT_QF_FACTOR: f64 = 5.0;
const DEFAULT_Q_FACTOR: f64 = 1.0;
const DEFAULT_R_FACTOR: f64 = 1.0;
const DEFAULT_TF: f64 = 5.0;
const DEFAULT_DT: f64 = 0.1;
const DEFAULT_MPC_HORIZON: f64 = 1.0;

#[derive(Clone, Debug)]
pub struct TrajOptions<S, I> {
    pub qf: DMatrix<f64>,
    pub q: DMatrix<f64>,
    pub r: DMatrix<f64>,
    pub noise_std: Option<Vec<f64>>,
    pub tf: f64,
    pub dt: f64,
    pub plot_flag: bool,
    pub x_goal: Vec<S>,
    pub u_goal: Vec<I>,
    pub x_op: Vec<S>,
    pub u_op: Vec<I>,
    pub finite_horizon_flag: bool,
    pub mpc_horizon: f64,
    pub estimated_parameters: Option<Vec<f64>>,
    pub u_limits: Option<ConstraintTransform>,
}

impl<S: Default + Clone + State, I: Default + Clone + State> TrajOptions<S, I> {
    pub fn new() -> Self {
        TrajOptions::<S, I>::default()
    }
    pub fn set_qf(self, qf: DMatrix<f64>) -> Self {
        let mut new = self;
        new.qf = qf;
        new
    }
    pub fn set_r(self, r: DMatrix<f64>) -> Self {
        let mut new = self;
        new.r = r;
        new
    }
    pub fn set_q(self, q: DMatrix<f64>) -> Self {
        let mut new = self;
        new.q = q;
        new
    }
    pub fn set_noise(self, noise_sources: Vec<f64>) -> Self {
        let mut new = self;
        new.noise_std = Some(noise_sources);
        new
    }
    pub fn set_tf(self, tf: f64) -> Self {
        let mut new = self;
        new.tf = tf;
        new
    }
    pub fn set_plot_flag(self, flag: bool) -> Self {
        let mut new = self;
        new.plot_flag = flag;
        new
    }
    pub fn set_x_goal(self, x_goal: Vec<S>) -> Self {
        let mut new = self;
        new.x_goal = x_goal;
        new
    }
    pub fn set_u_goal(self, u_goal: Vec<I>) -> Self {
        let mut new = self;
        new.u_goal = u_goal;
        new
    }
    pub fn set_x_op(self, x_op: Vec<S>) -> Self {
        let mut new = self;
        new.x_op = x_op;
        new
    }
    pub fn set_u_op(self, u_op: Vec<I>) -> Self {
        let mut new = self;
        new.u_op = u_op;
        new
    }
    pub fn set_finite_horizon(self, flag: bool) -> Self {
        let mut new = self;
        new.finite_horizon_flag = flag;
        new
    }
    pub fn set_estimated_parameters(self, params: Vec<f64>) -> Self {
        let mut new = self;
        new.estimated_parameters = Some(params);
        new
    }
    pub fn set_u_limits(self, constraints: ConstraintTransform) -> Self {
        let mut new = self;
        new.u_limits = Some(constraints);
        new
    }
    pub fn set_dt(self, dt: f64) -> Self {
        let mut new = self;
        new.dt = dt;
        new
    }
    pub fn set_mpc_horizon(self, horizon: f64) -> Self {
        let mut new = self;
        new.mpc_horizon = horizon;
        new
    }
}

impl<S, I> Default for TrajOptions<S, I>
where
    S: Default + Clone + State,
    I: Default + Clone + State,
{
    fn default() -> Self {
        let state_dims = S::dim_q();
        let input_dims = I::dim_q();
        Self {
            tf: DEFAULT_TF,
            qf: DMatrix::identity(state_dims, state_dims) * DEFAULT_QF_FACTOR,
            q: DMatrix::identity(state_dims, state_dims) * DEFAULT_Q_FACTOR,
            r: DMatrix::identity(input_dims, input_dims) * DEFAULT_R_FACTOR,
            noise_std: None,
            plot_flag: false,
            x_goal: vec![S::default(); 1],
            u_goal: vec![I::default(); 1],
            x_op: vec![S::default(); 1],
            u_op: vec![I::default(); 1],
            finite_horizon_flag: true,
            estimated_parameters: None,
            u_limits: None,
            mpc_horizon: DEFAULT_MPC_HORIZON,
            dt: DEFAULT_DT,
        }
    }
}
