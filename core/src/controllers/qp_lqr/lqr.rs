use super::utils;
use crate::controllers::{Controller, ControllerInput, ControllerState, CostFn, InputTrajectory};
use crate::physics::ModelError;
use crate::physics::discretizer::LinearDiscretizer;
use crate::physics::traits::{LinearDynamics, PhysicsSim, State};
use crate::solver::osqp::solver::OSQPSolverHandle;
use crate::solver::{Minimizer, OSQPBuilder};
use nalgebra::DVector;

pub struct QPLQR<S: PhysicsSim> {
    sim: S,
    solver: OSQPSolverHandle,

    u_traj: InputTrajectory<S>,

    n_steps: usize,
    dt: f64,
}

impl<S> QPLQR<S>
where
    S: PhysicsSim,
    S::Model: LinearDynamics,
    S::Discretizer: LinearDiscretizer<S::Model>,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        x0: &ControllerState<S>,
        u_limits: Option<(f64, f64)>,
        time_horizon: f64,
        dt: f64,
    ) -> Result<Self, ModelError> {
        if time_horizon <= 0.0 || dt <= 0.0 {
            return Err(ModelError::ConfigError(
                "Incorrect time configuration".into(),
            ));
        }
        let n_steps = (time_horizon / dt) as usize + 1;

        let u_traj = InputTrajectory(vec![ControllerInput::<S>::default(); n_steps - 1]);

        let jacobian_u = sim.discretizer().jacobian_u();
        let jacobian_x = sim.discretizer().jacobian_x();

        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let h = utils::build_h::<S>(&cost_fn, state_dim, input_dim, n_steps);
        let c = utils::build_c(jacobian_x, jacobian_u, n_steps);
        let d = utils::build_d(x0.to_vector(), jacobian_x, c.nrows());
        let q = DVector::zeros(h.ncols);
        let (g_opt, h_vec_opt) = if let Some((u_min_val, u_max_val)) = u_limits {
            let u_min = DVector::from_element(input_dim, u_min_val);
            let u_max = DVector::from_element(input_dim, u_max_val);
            let g = utils::build_g(input_dim, state_dim, n_steps);
            let h_vec = utils::build_h_vec(&u_min, &u_max, n_steps);
            (Some(g), Some(h_vec))
        } else {
            (None, None)
        };

        let mut qp_builder = OSQPBuilder::new();
        qp_builder = qp_builder.q_csc(h).q_vec(q).a_mat(c).b_vec(d);

        if let Some(g_mat) = g_opt {
            qp_builder = qp_builder.g_mat(g_mat);
        }
        if let Some(h_vec) = h_vec_opt {
            qp_builder = qp_builder.h_vec(h_vec);
        }

        let solver = qp_builder.build()?;

        Ok(QPLQR {
            sim,
            solver,
            u_traj,
            n_steps,
            dt,
        })
    }
}

impl<S: PhysicsSim> Controller<S> for QPLQR<S> {
    fn get_u_traj(&self) -> Vec<ControllerInput<S>> {
        self.u_traj.0.clone()
    }

    fn rollout(
        &self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerState<S>>, ModelError> {
        self.sim.rollout(
            initial_state,
            Some(self.u_traj.as_vec()),
            self.dt,
            self.n_steps,
        )
    }

    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<Vec<ControllerInput<S>>, ModelError> {
        let state_dim = ControllerState::<S>::dim_q() + ControllerState::<S>::dim_v();
        let input_dim = ControllerInput::<S>::dim_q();

        let mut x_traj = vec![initial_state.clone(); self.n_steps];
        let mut u_traj = vec![ControllerInput::<S>::default(); self.n_steps - 1];

        // retuls are in r.0 : [u1, x2, u2, ...]
        let r = self.solver.minimize(&[])?;

        for i in 0..self.n_steps - 1 {
            let base = i * (state_dim + input_dim);
            let next_state = ControllerState::<S>::from_slice(
                &r.0[base + input_dim..base + input_dim + state_dim],
            );
            let next_input = ControllerInput::<S>::from_slice(&r.0[base..base + input_dim]);
            x_traj[i + 1] = next_state;
            u_traj[i] = next_input;
        }
        self.u_traj = InputTrajectory::<S>(u_traj.clone());

        Ok(u_traj)
    }
}
