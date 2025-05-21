use nalgebra::{DMatrix, DVector};

use super::utils;
use crate::common::Labelizable;
use crate::numeric_services::solver::{KktConditionsStatus, NewtonSolverSymbolic, OptimizerConfig};
use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::traits::{Discretizer, LinearDynamics, State};
use crate::physics::{ModelError, constants as c};
use crate::solver::{LinearSolver, RootFinder};
use std::marker::PhantomData;
use std::sync::Arc;

// Backward Euler residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - dt * f(x_{k+1})
//
// The root of R(x_{k+1}) gives the next state satisfying the Backward Euler integration scheme.

pub struct BackwardEuler<D: SymbolicDynamics> {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolverSymbolic,
    _phantom_data: PhantomData<D>,
}

impl<D: SymbolicDynamics> BackwardEuler<D> {
    pub fn new(
        model: &D,
        registry: Arc<ExprRegistry>,
        solver_options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let solver;
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let (current_state, next_state) = utils::get_states(&registry)?;

        if let Some(linear_term) = model.cost_linear_term(&dt_expr, &registry) {
            solver = utils::init_constrained_dynamics(
                &linear_term,
                &dt_expr,
                solver_options,
                &registry,
            )?;
        } else {
            let dyn_next_state = model
                .dynamics_symbolic(&next_state.wrap(), &registry)
                .scale(&dt_expr);

            let residual = next_state.sub(&current_state).sub(&dyn_next_state);
            solver = NewtonSolverSymbolic::new_root_solver(
                &residual,
                &next_state,
                &registry,
                solver_options,
            )?;
        }

        Ok(BackwardEuler {
            registry,
            solver,
            _phantom_data: PhantomData,
        })
    }

    pub fn solver_status(&self) -> &Option<KktConditionsStatus> {
        self.solver.status()
    }
}

impl<D: SymbolicDynamics> Discretizer<D> for BackwardEuler<D> {
    fn step(
        &mut self,
        _model: &D,
        state: &D::State,
        _input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let v_dims = D::State::dim_v();
        let (next_v, _mus, _lambdas) =
            utils::step_intrinsic(state, dt, &mut self.solver, &self.registry)?;
        if v_dims == 0 {
            return Ok(D::State::from_vec(next_v));
        }

        let labels = D::State::labels();
        let q_slice = &labels[..D::State::dim_q()];

        let current_q = state.vectorize(q_slice);
        let next_q: Vec<_> = current_q
            .iter()
            .zip(next_v.iter())
            .map(|(q, v)| q + dt * v)
            .collect();
        let mut full_state = next_q;
        full_state.extend_from_slice(&next_v);

        Ok(D::State::from_vec(full_state))
    }
}

pub struct BackwardEulerLinear<D: LinearDynamics> {
    _phantom_data: PhantomData<D>,
}

impl<D: LinearDynamics> BackwardEulerLinear<D> {
    pub fn new() -> Self {
        Self {
            _phantom_data: PhantomData,
        }
    }
}

impl<D: LinearDynamics> Discretizer<D> for BackwardEulerLinear<D> {
    fn step(
        &mut self,
        model: &D,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let state_mat = model.get_state_slice();
        let control_mat = model.get_control_slice();
        let input_dims = control_mat.ncols();
        let input = DVector::from_vec((input.unwrap_or(&vec![0.0; input_dims])).to_owned());
        let state = DVector::from_vec(state.to_vec());

        let j = DMatrix::identity(state_mat.nrows(), state_mat.ncols()) - dt * state_mat;
        let rhs = &state + dt * model.get_control_slice() * input;
        let residual: Box<dyn Fn(&DVector<f64>) -> DVector<f64>> =
            Box::new(|x_next: &DVector<f64>| j.clone() * x_next - rhs.clone());

        let mut solver = LinearSolver::new(&residual, &j);
        let r = solver.find_roots(state.as_slice())?;
        Ok(D::State::from_vec(r.0))
    }

}
