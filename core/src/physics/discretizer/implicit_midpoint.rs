use super::utils;
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::models::state::State;
use crate::physics::traits::{Discretizer, Dynamics};
use crate::physics::{ModelError, constants as c};
use solvers::NewtonSolverSymbolic;
use solvers::dtos::OptimizerConfig;
use symbolic_services::symbolic::{ExprRegistry, ExprScalar};
use crate::utils::Labelizable;
use std::marker::PhantomData;
use std::sync::Arc;

// residual(x_next) = x_next - x_k - dt * f((x_k + x_next) / 2)

const DEFAULT_TOLERANCE: f64 = 1e-3;
pub struct ImplicitMidpoint<D: Dynamics> {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolverSymbolic,
    tol: f64,
    _phantom_data: PhantomData<D>,
}

impl<D: SymbolicDynamics> ImplicitMidpoint<D> {
    pub fn new(
        model: &D,
        registry: Arc<ExprRegistry>,
        solver_options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let solver;
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let (current_state, next_state) = utils::get_states(&registry)?;

        let tol = solver_options
            .as_ref()
            .map_or(DEFAULT_TOLERANCE, |options| options.get_tolerance());

        if let Some(linear_term) = model.cost_linear_term(&dt_expr, &registry) {
            solver = utils::init_constrained_dynamics(
                &linear_term,
                &dt_expr,
                solver_options,
                &registry,
            )?;
        } else {
            let mid_state = current_state.add(&next_state).wrap().scalef(0.5).wrap();

            let dyn_mid_state = model
                .dynamics_symbolic(&mid_state, &registry)
                .wrap()
                .scale(&dt_expr);

            let residual = next_state.sub(&current_state).sub(&dyn_mid_state).wrap();
            solver = NewtonSolverSymbolic::new_root_solver(
                &residual,
                &next_state,
                &registry,
                solver_options,
            )?;
        }

        Ok(ImplicitMidpoint {
            registry,
            solver,
            tol,
            _phantom_data: PhantomData,
        })
    }

    pub fn set_tolerance(&mut self, tol: f64) {
        self.tol = tol;
    }
}

impl<D: Dynamics> Discretizer<D> for ImplicitMidpoint<D> {
    fn step(
        &self,
        _model: &D,
        state: &D::State,
        _input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let v_dims = D::State::dim_v();
        let (next_v, status, _mus, _lambdas) =
            utils::step_intrinsic(state, dt, &self.solver, &self.registry)?;
        if v_dims == 0 {
            return Ok(D::State::from_vec(next_v));
        }
        utils::check_convergence(status, self.tol)?;

        let labels = D::State::labels();
        let q_slice = &labels[..D::State::dim_q()];
        let v_slice = &labels[D::State::dim_q()..];

        let current_q = state.vectorize(q_slice);
        let current_v = state.vectorize(v_slice);
        let next_q: Vec<_> = current_q
            .iter()
            .zip(current_v.iter())
            .zip(next_v.iter())
            .map(|((q, v_curr), v_next)| q + dt * 0.5 * (v_curr + v_next))
            .collect();
        let mut full_state = next_q;
        full_state.extend_from_slice(&next_v);

        Ok(D::State::from_vec(full_state))
    }
}
