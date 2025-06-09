use nalgebra::{DMatrix, DVector};

use super::options::QPOptions;
use super::utils;
use crate::controllers::{
    Controller, ControllerInput, ControllerState, CostFn, SteppableController, TrajectoryHistory,
    UpdatableController,
};
use crate::physics::ModelError;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim, State};
use crate::solver::osqp::builder::QPParams;
use crate::solver::osqp::solver::OSQPSolverHandle;
use crate::solver::{Minimizer, OSQPBuilder};
use crate::utils::evaluable::EvaluableDMatrix;
use crate::utils::{matrix, vector};

pub struct QPLQRGeneric<S: PhysicsSim> {
    #[allow(dead_code)]
    sim: S,
    solver: OSQPSolverHandle,

    n_steps: usize,
    dt: f64,

    u_ref: Vec<ControllerInput<S>>,
    x_ref: Vec<ControllerState<S>>,

    control_mat: DMatrix<f64>,
    state_mat: DMatrix<f64>,

    cost_fn: CostFn<S>,
}

impl<S> QPLQRGeneric<S>
where
    S: PhysicsSim,
    S::Model: Dynamics,
    S::Discretizer: Discretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        jacobian_x_fn: EvaluableDMatrix,
        jacobian_u_fn: EvaluableDMatrix,
        x0: &ControllerState<S>,
        options: QPOptions<S>,
    ) -> Result<(Self, QPParams), ModelError> {
        let n_steps = (options.get_general().get_time_horizon() / options.get_general().get_dt())
            as usize
            + 1;

        let vals = options.general.concatenate_operating_point();

        let control_mat = jacobian_u_fn.evaluate(&vals)?;
        let state_mat = jacobian_x_fn.evaluate(&vals)?;

        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let u_ref = options.general.get_u_ref().to_vec();
        let x_ref = options.general.get_x_ref().to_vec();

        // quadratic cost matrix => 0.5 * x' * H * x
        let h = utils::build_h::<S>(&cost_fn, state_dim, input_dim, n_steps - 1);
        // linear cost matrix => q' * x
        let q = utils::build_q_vec::<S>(&cost_fn, &x_ref, n_steps - 1);
        // equality matrix => C * x = d
        let mut c = utils::build_c(&state_mat, &control_mat, n_steps - 1);
        let mut lb_vec = utils::build_d(x0.to_vector(), &state_mat, c.nrows());
        let mut ub_vec = lb_vec.clone();

        // inequality input matrix => lb <= g * input <= ub; c = [c; g]
        if let Some(input_constraints) = options.general.get_u_limits() {
            let (lb, g_mat, ub) = input_constraints.expand_input::<S>(n_steps - 1)?;
            c = matrix::vstack_option(c, Some(g_mat))
                .map_err(|e| ModelError::Other(e.to_string()))?;
            lb_vec = vector::vstack_option(lb_vec, Some(lb));
            ub_vec = vector::vstack_option(ub_vec, Some(ub));
        }

        // inequality state matrix => lb <= g * state <= ub; c = [c; g]
        if let Some(state_constraints) = options.general.get_x_limits() {
            let (lb, g_mat, ub) = state_constraints.expand_state::<S>(n_steps - 1)?;
            c = matrix::vstack_option(c, Some(g_mat))
                .map_err(|e| ModelError::Other(e.to_string()))?;
            lb_vec = vector::vstack_option(lb_vec, Some(lb));
            ub_vec = vector::vstack_option(ub_vec, Some(ub));
        }

        let mut qp_builder = OSQPBuilder::new();
        qp_builder = qp_builder
            .q_csc(h.clone())
            .q_vec(q.clone())
            .a_mat(c)
            .bounds_vec(lb_vec, ub_vec)
            .add_options(options.get_osqp_settings().clone());

        let (solver, updatable_qp_params) = qp_builder.build()?;

        Ok((
            QPLQRGeneric {
                sim,
                solver,
                n_steps,
                u_ref,
                x_ref,
                dt: options.get_general().get_dt(),
                state_mat,   // A
                control_mat, // B
                cost_fn,     // can get Q, Qn, R
            },
            updatable_qp_params,
        ))
    }

    pub fn update(&self, builder: OSQPBuilder) {
        builder.update(&self.solver)
    }
}

impl<S: PhysicsSim> UpdatableController<S> for QPLQRGeneric<S> {
    type Params<'a> = OSQPBuilder<'a>;

    fn update(&self, params: Self::Params<'_>) {
        params.update(&self.solver)
    }

    fn update_bounds(
        &self,
        current_state: &DVector<f64>,
        lb: &mut DVector<f64>,
        ub: &mut DVector<f64>,
    ) {
        // update vector d[0] to -A * x0; => lb <= Az <= ub;
        let a_x = -&self.state_mat * current_state;
        lb.rows_mut(0, current_state.len()).copy_from(&a_x);
        ub.rows_mut(0, current_state.len()).copy_from(&a_x);
    }

    fn update_q(&self, state_ref: &DVector<f64>, q: &mut DVector<f64>) {
        let n_steps = self.n_steps;
        let state_dims = state_ref.len();
        let input_dims = ControllerInput::<S>::dim_q();

        if let (Some(running_cost), Some(terminal_cost)) =
            (self.cost_fn.get_q(), self.cost_fn.get_qn())
        {
            let q_x = -running_cost * state_ref;
            let qn_x = -terminal_cost * state_ref;

            for j in 0..n_steps - 2 {
                let offset = input_dims + j * (state_dims + input_dims);
                q.rows_mut(offset, state_dims).copy_from(&q_x);
            }

            // Final step (for j = Nh)
            let offset = input_dims + (n_steps - 2) * (state_dims + input_dims);
            q.rows_mut(offset, state_dims).copy_from(&qn_x);
        }
    }
}

impl<S: PhysicsSim> Controller<S> for QPLQRGeneric<S> {
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];
        let mut x_traj = vec![initial_state.clone(); self.n_steps];

        let dt = self.dt;

        // retuls are in r.0 : [u1, x2, u2, ...]
        let r = self.solver.minimize(&[])?;

        for i in 0..self.n_steps - 1 {
            let base = i * (state_dim + input_dim);
            let next_input = ControllerInput::<S>::from_slice(&r.0[base..base + input_dim]);
            u_traj[i] = next_input + self.u_ref[0].clone();
            x_traj[i + 1] = self.sim.step(&x_traj[i], Some(&u_traj[i]), dt)?;
        }
        Ok((x_traj, u_traj))
    }
}

impl<S: PhysicsSim> SteppableController<S> for QPLQRGeneric<S> {
    fn step(
        &self,
        state: ControllerState<S>,
        input: Option<&ControllerInput<S>>,
        dt: f64,
    ) -> Result<ControllerState<S>, ModelError> {
        self.sim.step(&state, input, dt)
    }
}
