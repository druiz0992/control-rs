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
