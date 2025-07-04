use crate::controllers::ddp::DDPOptions;
use crate::controllers::utils::{linearize_one, linearize_second_order_one};
use crate::controllers::{Controller, ControllerInput, ControllerState, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::NumericDiscretizer;
use crate::physics::{models::Dynamics, traits::State};
use crate::utils::Labelizable;
use crate::utils::evaluable::EvaluableMatrixFn;
use crate::{controllers::CostFn, physics::traits::PhysicsSim};
use general::matrix::compute_comm_kron_product;
use nalgebra::{DMatrix, DVector, SymmetricEigen};

type CostExpansion = (DMatrix<f64>, DVector<f64>);
type BackwardPassData = (f64, Vec<DVector<f64>>, Vec<DMatrix<f64>>);

#[derive(Clone, Debug)]
struct DDPStats {
    alpha: f64,
    cost_to_go_n: f64,
}

pub struct DDP<S: PhysicsSim> {
    sim: S,
    cost_fn: CostFn<S>,

    jacobian_u_fn: EvaluableMatrixFn,
    jacobian_x_fn: EvaluableMatrixFn,
    hessian_xx_fn: Option<EvaluableMatrixFn>,
    hessian_xu_fn: Option<EvaluableMatrixFn>,
    hessian_ux_fn: Option<EvaluableMatrixFn>,
    hessian_uu_fn: Option<EvaluableMatrixFn>,

    u_traj: Vec<ControllerInput<S>>,
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
        let u_traj = options.get_general().get_u_ref().to_owned();

        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        let jacobian_u_fn = sim.discretizer().jacobian_u();
        let jacobian_x_fn = sim.discretizer().jacobian_x();
        let hessian_xx_fn = sim.discretizer().hessian_xx();
        let hessian_xu_fn = sim.discretizer().hessian_xu();
        let hessian_ux_fn = sim.discretizer().hessian_ux();
        let hessian_uu_fn = sim.discretizer().hessian_uu();

        Ok(Self {
            sim,
            cost_fn,
            jacobian_u_fn,
            jacobian_x_fn,
            hessian_uu_fn,
            hessian_ux_fn,
            hessian_xu_fn,
            hessian_xx_fn,
            options,
            n_steps,
            u_traj,
        })
    }

    fn backward_pass(&self, x_traj: &[ControllerState<S>]) -> Result<BackwardPassData, ModelError> {
        let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let nu = ControllerInput::<S>::dim_q();

        let mut delta_cost = 0.0;
        // P
        let mut cost_quadratic_term = vec![DMatrix::<f64>::zeros(nx, nx); self.n_steps];
        // p
        let mut cost_linear_term = vec![DVector::<f64>::zeros(nx); self.n_steps];
        // K
        let mut feedback_gain = vec![DMatrix::<f64>::zeros(nu, nx); self.n_steps - 1];
        // d
        let mut feedforward_control = vec![DVector::<f64>::zeros(nu); self.n_steps - 1];

        // P[N], p[N] =  Qn, Qn * (x_traj[N] - x_goal)
        (
            cost_quadratic_term[self.n_steps - 1],
            cost_linear_term[self.n_steps - 1],
        ) = self.terminal_cost_expansion(x_traj)?;

        let general_options = self
            .options
            .get_general()
            .to_owned()
            .set_u_operating(&self.u_traj)
            .set_x_operating(x_traj);

        for k in (0..self.n_steps - 1).rev() {
            let (a_mat, b_mat) = linearize_one(
                &self.sim,
                &self.jacobian_x_fn,
                &self.jacobian_u_fn,
                k,
                &general_options,
            )?;
            let a_mat_t = &a_mat.transpose();
            let b_mat_t = &b_mat.transpose();
            let ((j_xx, j_x), (j_uu, j_u)) = self.stage_cost_expansion(x_traj, &self.u_traj, k)?;

            let a_mat_t_quadratic = a_mat_t * &cost_quadratic_term[k + 1];
            let b_mat_t_quadratic = b_mat_t * &cost_quadratic_term[k + 1];

            let g_x = j_x + a_mat_t * &cost_linear_term[k + 1];
            let g_u = j_u + b_mat_t * &cost_linear_term[k + 1];

            let mut g_xx = j_xx + &a_mat_t_quadratic * &a_mat;
            let mut g_uu = j_uu + &b_mat_t_quadratic * &b_mat;
            let mut g_ux = &b_mat_t_quadratic * &a_mat;
            let mut g_xu = &a_mat_t_quadratic * &b_mat;

            if !self.options.get_ilqr_enable() {
                if let (
                    Some(hessian_xx_fn),
                    Some(hessian_xu_fn),
                    Some(hessian_ux_fn),
                    Some(hessian_uu_fn),
                ) = (
                    &self.hessian_xx_fn,
                    &self.hessian_xu_fn,
                    &self.hessian_ux_fn,
                    &self.hessian_uu_fn,
                ) {
                    let (a_x, a_u, b_x, b_u) = linearize_second_order_one(
                        &self.sim,
                        hessian_xx_fn,
                        hessian_xu_fn,
                        hessian_ux_fn,
                        hessian_uu_fn,
                        k,
                        &general_options,
                    )?;

                    g_xx += compute_comm_kron_product(&cost_linear_term[k + 1], &a_x, nx);
                    g_uu += compute_comm_kron_product(&cost_linear_term[k + 1], &b_u, nu);
                    g_ux += compute_comm_kron_product(&cost_linear_term[k + 1], &b_x, nu);
                    g_xu += compute_comm_kron_product(&cost_linear_term[k + 1], &a_u, nx);
                    regularize_g(&mut g_xx, &g_xu, &g_ux, &mut g_uu);
                }
            }

            let lu = g_uu.clone().lu();
            feedforward_control[k] = lu.solve(&g_u).expect("Matrix is singular!");
            feedback_gain[k] = lu.solve(&g_ux).expect("Matrix is singular!");

            let feedback_gain_t = &feedback_gain[k].transpose();

            cost_quadratic_term[k] = &g_xx + feedback_gain_t * &g_uu * &feedback_gain[k]
                - &g_xu * &feedback_gain[k]
                - feedback_gain_t * &g_ux;
            cost_linear_term[k] = &g_x - feedback_gain_t * &g_u
                + feedback_gain_t * &g_uu * &feedforward_control[k]
                - &g_xu * &feedforward_control[k];

            delta_cost += (g_u.transpose() * &feedforward_control[k])[(0, 0)];
        }

        Ok((delta_cost, feedforward_control, feedback_gain))
    }

    fn forward_pass(
        &mut self,
        x_traj: &[ControllerState<S>],
        feedforward_control: &[DVector<f64>],
        feedback_gain: &[DMatrix<f64>],
    ) -> Result<(Vec<ControllerState<S>>, DDPStats), ModelError> {
        let dt = self.options.get_general().get_dt();

        let mut alpha = 1.0;
        let cost_to_go = self.cost_fn.total_cost(x_traj, &self.u_traj)?;
        let mut x_new = x_traj.to_owned().clone();
        let mut u_new = self.u_traj.clone();

        for _ in 0..self.options.get_max_iters_linesearch() {
            for k in 0..self.n_steps - 1 {
                let u = self.u_traj[k].to_vector()
                    - alpha * &feedforward_control[k]
                    - &feedback_gain[k] * (&x_new[k].to_vector() - x_traj[k].to_vector());
                u_new[k] = ControllerInput::<S>::from_slice(u.as_slice());
                x_new[k + 1] = self.sim.step(&x_new[k], Some(&u_new[k]), dt)?;
            }
            let cost_to_go_n = self.cost_fn.total_cost(&x_new, &u_new)?;

            if cost_to_go_n < cost_to_go {
                self.u_traj = u_new;
                return Ok((
                    x_new,
                    DDPStats {
                        alpha,
                        cost_to_go_n,
                    },
                ));
            } else {
                alpha *= 0.5;
            }
        }

        Err(ModelError::SolverError("Max iterations reached!".into()))
    }

    /// if terminal cost is Jn(x,u), return grad^2_x Jn(x,u), grad_x Jn(x,u)
    fn terminal_cost_expansion(
        &self,
        state: &[ControllerState<S>],
    ) -> Result<CostExpansion, ModelError> {
        if state.is_empty() {
            return Err(ModelError::Unexpected(
                "State vector cannot be empty".into(),
            ));
        }
        let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let dstate_cost_dxx = self
            .cost_fn
            .get_qn()
            .unwrap_or(&DMatrix::<f64>::zeros(nx, nx))
            .to_owned();
        let terminal_cost_gradient = self.cost_fn.terminal_cost_gradient(state.last().unwrap());

        Ok((dstate_cost_dxx, terminal_cost_gradient))
    }

    /// if stage cost is J(x,u), return ((grad^2_x J(x,u), grad_x J(x,u)), (grad^2_u J(x,u), grad_u J(x,u)))
    fn stage_cost_expansion(
        &self,
        state: &[ControllerState<S>],
        input: &[ControllerInput<S>],
        stage: usize,
    ) -> Result<(CostExpansion, CostExpansion), ModelError> {
        if state.is_empty() || input.is_empty() {
            return Err(ModelError::Unexpected(
                "State/Input vectors cannot be empty".into(),
            ));
        }
        let nx = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let nu = ControllerInput::<S>::dim_q();
        let dstate_cost_dxx = self
            .cost_fn
            .get_q()
            .unwrap_or(&DMatrix::<f64>::zeros(nx, nx))
            .to_owned();
        let dinput_cost_duu = self
            .cost_fn
            .get_r()
            .unwrap_or(&DMatrix::<f64>::zeros(nu, nu))
            .to_owned();
        let (state_cost_gradient, input_cost_gradient) =
            self.cost_fn
                .stage_cost_gradient(&state[stage], &input[stage], stage)?;

        Ok((
            (dstate_cost_dxx, state_cost_gradient),
            (dinput_cost_duu, input_cost_gradient),
        ))
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

        let mut x_traj = self
            .sim
            .rollout(initial_state, Some(&self.u_traj), dt, self.n_steps)?;

        let mut stats;
        for niter in 0..max_iters {
            let (delta_cost, feedforward_control, feedback_gain) = self.backward_pass(&x_traj)?;
            (x_traj, stats) = self.forward_pass(&x_traj, &feedforward_control, &feedback_gain)?;

            if delta_cost < self.options.get_tol() {
                if self.options.get_verbose() {
                    println!("ILQR converged");
                }
                return Ok((x_traj, self.u_traj.clone()));
            }

            if self.options.get_verbose() {
                let max_feedforward_norm = feedforward_control
                    .iter()
                    .map(|v| v.norm())
                    .fold(0.0, |a: f64, b: f64| a.max(b));
                println!(
                    "iter: {}\t cost_to_go:{:.4}\t delta_cost: {:.4}\t |d|: {:.4}\t alpha: {:.4}",
                    niter, stats.cost_to_go_n, delta_cost, max_feedforward_norm, stats.alpha
                );
            }
        }
        Ok((x_traj, self.u_traj.clone()))
    }
}

fn regularize_g(
    g_xx: &mut DMatrix<f64>,
    g_xu: &DMatrix<f64>,
    g_ux: &DMatrix<f64>,
    g_uu: &mut DMatrix<f64>,
) {
    let nx = g_xx.nrows();
    let nu = g_uu.nrows();

    let mut beta = 0.1;
    loop {
        // Construct symmetric block matrix:
        // [Gxx  Gxu]
        // [Gux  Guu]
        let mut big_mat = DMatrix::<f64>::zeros(nx + nu, nx + nu);

        // Top-left block: Gxx
        big_mat.view_mut((0, 0), (nx, nx)).copy_from(&*g_xx);
        // Top-right block: Gxu
        big_mat.view_mut((0, nx), (nx, nu)).copy_from(g_xu);
        // Bottom-left block: Gux
        big_mat.view_mut((nx, 0), (nu, nx)).copy_from(g_ux);
        // Bottom-right block: Guu
        big_mat.view_mut((nx, nx), (nu, nu)).copy_from(&*g_uu);

        // Check if symmetric positive definite
        // Here we check if all eigenvalues > 0
        let eig = SymmetricEigen::new(big_mat.clone());
        if eig.eigenvalues.iter().all(|&ev| ev > 0.0) {
            break;
        }

        // Not positive definite: regularize Gxx and Guu by adding beta * I
        for i in 0..nx {
            g_xx[(i, i)] += beta;
        }
        for i in 0..nu {
            g_uu[(i, i)] += beta;
        }

        beta *= 2.0;
    }
}
