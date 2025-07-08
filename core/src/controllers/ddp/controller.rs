use crate::controllers::ddp::DDPOptions;
use crate::controllers::ddp::q_hessian::Q;
use crate::controllers::ddp::utils;
use crate::controllers::utils::clamp_input_vector;
use crate::controllers::{Controller, ControllerInput, ControllerState, TrajectoryHistory};
use crate::controllers::{HessianFns, JacobianFns};
use crate::physics::ModelError;
use crate::physics::discretizer::NumericDiscretizer;
use crate::physics::{models::Dynamics, traits::State};
use crate::utils::Labelizable;
use crate::{controllers::CostFn, physics::traits::PhysicsSim};
use nalgebra::{DMatrix, DVector};
use solvers::osqp::OSQPCluster;

const ALPHA_LINESEARCH: f64 = 1.0;

/// Stores DDP Stats
#[derive(Clone, Debug)]
struct Stats {
    /// linesearch coeff
    alpha: f64,
    /// measured cost to go
    cost_to_go_n: f64,
}

pub struct DDP<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,

    jacobian_fns: JacobianFns,
    hessian_fns: Option<HessianFns>,

    osqp_cluster: OSQPCluster,

    feedforward_control: Vec<DVector<f64>>,
    feedback_gain: Vec<DMatrix<f64>>,

    u_traj: Vec<ControllerInput<S>>,
    x_traj: Vec<ControllerState<S>>,

    n_steps: usize,
    options: DDPOptions<S>,
}

impl<S> DDP<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: NumericDiscretizer<S::Model>,
{
    pub fn new_numeric(
        sim: S,
        cost_fn: CostFn<S>,
        options: DDPOptions<S>,
    ) -> Result<Self, ModelError> {
        let nu = ControllerInput::<S>::dim_q();
        let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let u_traj = options.get_general().get_u_ref().to_owned();

        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;
        let x_traj = vec![ControllerState::<S>::default(); n_steps];

        let jacobian_fns = JacobianFns::from_sim(&sim);
        let hessian_fns = HessianFns::from_sim(&sim);

        let osqp_settings = options.get_osqp_settings();
        let mut osqp_cluster = OSQPCluster::new(n_steps - 1, Some(osqp_settings));
        for k in 0..n_steps - 1 {
            osqp_cluster.initialize(k, nu)?;
        }
        // K
        let feedback_gain = vec![DMatrix::<f64>::zeros(nu, nx); n_steps - 1];
        // d
        let feedforward_control = vec![DVector::<f64>::zeros(nu); n_steps - 1];

        Ok(Self {
            sim,
            cost_fn,
            jacobian_fns,
            hessian_fns,
            options,
            n_steps,
            u_traj,
            x_traj,
            osqp_cluster,
            feedback_gain,
            feedforward_control,
        })
    }

    fn backward_pass(&mut self) -> Result<f64, ModelError> {
        let x_traj = &self.x_traj;
        let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();

        let mut delta_cost = 0.0;
        // P
        let mut cost_quadratic_term = vec![DMatrix::<f64>::zeros(nx, nx); self.n_steps];
        // p
        let mut cost_linear_term = vec![DVector::<f64>::zeros(nx); self.n_steps];

        // P[N], p[N] =  Qn, Qn * (x_traj[N] - x_goal)
        (
            cost_quadratic_term[self.n_steps - 1],
            cost_linear_term[self.n_steps - 1],
        ) = utils::terminal_cost_expansion::<S>(x_traj, &self.cost_fn)?;

        // update operating points
        let general_options = utils::update_operating_points(x_traj, &self.u_traj, &self.options);

        for k in (0..self.n_steps - 1).rev() {
            // linearize dynamics into A and B for stage k
            let (a_mat, b_mat) =
                self.jacobian_fns
                    .linearize_step(&self.sim, k, &general_options)?;
            let a_mat_t = &a_mat.transpose();
            let b_mat_t = &b_mat.transpose();
            let ((l_xx, l_x), (l_uu, l_u)) =
                utils::stage_cost_expansion::<S>(x_traj, &self.u_traj, k, &self.cost_fn)?;

            let a_mat_t_quadratic = a_mat_t * &cost_quadratic_term[k + 1];
            let b_mat_t_quadratic = b_mat_t * &cost_quadratic_term[k + 1];

            let q_x = l_x + a_mat_t * &cost_linear_term[k + 1];
            let q_u = l_u + b_mat_t * &cost_linear_term[k + 1];

            let q_xx = l_xx + &a_mat_t_quadratic * &a_mat;
            let q_uu = l_uu + &b_mat_t_quadratic * &b_mat;
            let q_ux = &b_mat_t_quadratic * &a_mat;
            let q_xu = &a_mat_t_quadratic * &b_mat;

            let mut q_hessian = Q::new(q_xx, q_xu, q_ux, q_uu);

            (self.feedback_gain[k], self.feedforward_control[k]) =
                q_hessian.compute_gains_via_inverse(&q_u);

            // Apply second order term corrections if DDP enabled. DDP doesnt allow u constraints
            if self.options.get_ddp_enable() {
                if let Some(hessian_fns) = &self.hessian_fns {
                    let (a_x, a_u, b_x, b_u) =
                        hessian_fns.linearize_second_order_step(&self.sim, k, &general_options)?;

                    q_hessian = q_hessian
                        + Q::from_second_order_terms(a_x, a_u, b_x, b_u, &cost_linear_term[k + 1]);
                    (self.feedback_gain[k], self.feedforward_control[k]) =
                        q_hessian.compute_unconstrained_gains(&q_u);
                }
            // apply u constraints if enabled and ILQR
            } else if let Some(u_limits) = self.options.get_general().get_u_limits() {
                let (lb, _, ub) = u_limits.expand_input::<S>(1)?;
                let u_nominal = self.u_traj[k].to_vector();
                let delta_lb = &lb - &u_nominal;
                let delta_ub = &ub - &u_nominal;
                let delta_u = self.osqp_cluster.solve_step(
                    k,
                    q_hessian.q_uu(),
                    &q_u,
                    &delta_lb,
                    &delta_ub,
                )?;

                self.feedback_gain[k] =
                    q_hessian.compute_constrained_gains(&delta_u, &delta_lb, &delta_ub);
                self.feedforward_control[k] = -delta_u;
            }

            (cost_quadratic_term[k], cost_linear_term[k]) = q_hessian.compute_cost_terms(
                &q_x,
                &q_u,
                &self.feedback_gain[k],
                &self.feedforward_control[k],
            );

            delta_cost += (q_u.transpose() * &self.feedforward_control[k])[(0, 0)]
                + 0.5
                    * (&self.feedforward_control[k].transpose()
                        * q_hessian.q_uu()
                        * &self.feedforward_control[k])[(0, 0)];
        }

        Ok(delta_cost)
    }

    fn rollout(
        &self,
        x_traj: &[ControllerState<S>],
        x_new: &mut [ControllerState<S>],
        u_new: &mut [ControllerInput<S>],
        alpha: f64,
    ) -> Result<(), ModelError> {
        let dt = self.options.get_general().get_dt();
        for k in 0..self.n_steps - 1 {
            let u = self.u_traj[k].to_vector()
                - alpha * &self.feedforward_control[k]
                - &self.feedback_gain[k] * (&x_new[k].to_vector() - x_traj[k].to_vector());
            let u_clamped = clamp_input_vector(u, self.options.get_general().get_u_limits());

            u_new[k] = ControllerInput::<S>::from_slice(u_clamped.as_slice());
            x_new[k + 1] = self.sim.step(&x_new[k], Some(&u_new[k]), dt)?;
        }
        Ok(())
    }

    fn forward_pass(
        &mut self,
    ) -> Result<Stats, ModelError> {
        let x_traj = &self.x_traj;
        let mut alpha = ALPHA_LINESEARCH;
        let cost_to_go = self.cost_fn.total_cost(x_traj, &self.u_traj)?;
        let mut x_new = x_traj.to_owned().clone();
        let mut u_new = self.u_traj.clone();

        for _ in 0..self.options.get_max_iters_linesearch() {
            self.rollout(x_traj, &mut x_new, &mut u_new, alpha)?;
            let cost_to_go_n = self.cost_fn.total_cost(&x_new, &u_new)?;

            if cost_to_go_n < cost_to_go {
                self.u_traj = u_new;
                self.x_traj = x_new;
                return Ok(
                    Stats {
                        alpha,
                        cost_to_go_n,
                    },
                );
            } else {
                alpha *= 0.5;
            }
        }
        Ok(
            Stats {
                alpha: 0.0,
                cost_to_go_n: 0.0,
            },
        )
    }
}

impl<S> Controller<S> for DDP<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: NumericDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let max_iters = self.options.get_max_iters();
        let dt = self.options.get_general().get_dt();

        self.x_traj = self
            .sim
            .rollout(initial_state, Some(&self.u_traj), dt, self.n_steps)?;

        let mut stats;
        let mut last_delta_cost = 0.0;
        for niter in 0..max_iters {
            let delta_cost = self.backward_pass()?;
            stats = self.forward_pass()?;

            if self.options.get_verbose() {
                let max_feedforward_norm = self
                    .feedforward_control
                    .iter()
                    .map(|v| v.norm())
                    .fold(0.0, |a: f64, b: f64| a.max(b));
                println!(
                    "iter: {}\t cost_to_go:{:.4}\t delta_cost: {:.4}\t |d|: {:.4}\t alpha: {:.4}",
                    niter, stats.cost_to_go_n, delta_cost, max_feedforward_norm, stats.alpha
                );
            }

            if delta_cost < self.options.get_tol() {
                if self.options.get_verbose() {
                    println!("ILQR converged");
                }
                break;
            }
            if (delta_cost - last_delta_cost).abs() < self.options.get_tol() {
                break;
            }
            last_delta_cost = delta_cost;
        }
        Ok((self.x_traj.clone(), self.u_traj.clone()))
    }
}
