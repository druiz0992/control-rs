use crate::physics::models::dynamics::SymbolicDynamics;
use crate::physics::models::state::State;
use crate::physics::traits::{Discretizer, Dynamics};
use crate::physics::{ModelError, constants as c};
use crate::utils::Labelizable;
use solvers::NewtonSolverSymbolic;
use solvers::dtos::OptimizerConfig;
use std::marker::PhantomData;
use std::sync::Arc;
use symbolic_services::symbolic::{ExprRegistry, ExprScalar};

use super::utils::{get_states, step_intrinsic};

// Hermite–Simpson residual:
//
// R(x_{k+1}) = x_{k+1} - x_k - (dt / 6) * (f_k + 4 * f_m + f_{k+1})
//
// where:
//   f_k     = f(x_k)
//   f_{k+1} = f(x_{k+1})
//   x_m     = 0.5 * (x_k + x_{k+1}) + (dt / 8) * (f_k - f_{k+1})
//   f_m     = f(x_m)
//
// The root of R(x_{k+1}) gives the next state satisfying the Hermite–Simpson integration scheme.

pub struct HermiteSimpson<D: Dynamics> {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolverSymbolic,
    _phantom_data: PhantomData<D>,
}

impl<D: SymbolicDynamics> HermiteSimpson<D> {
    pub fn new(
        model: &D,
        registry: Arc<ExprRegistry>,
        solver_options: Option<OptimizerConfig>,
    ) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let dt6 = dt_expr.scalef(1.0 / 6.0);
        let dt8 = dt_expr.scalef(1.0 / 8.0);
        let (current_state, next_state) = get_states(&registry)?;

        let dyn_current_state = model
            .dynamics_symbolic(&current_state.wrap(), &registry)
            .wrap();

        let dyn_next_state = model
            .dynamics_symbolic(&next_state.wrap(), &registry)
            .wrap();

        let mid_state = dyn_current_state
            .sub(&dyn_next_state)
            .wrap()
            .scale(&dt8)
            .add(&current_state.add(&next_state).wrap().scalef(0.5));

        let dyn_mid_state = model.dynamics_symbolic(&mid_state.wrap(), &registry).wrap();

        let mut residual = dyn_current_state
            .add(&dyn_mid_state.scalef(4.0))
            .add(&dyn_next_state)
            .wrap()
            .scale(&dt6);
        residual = current_state.add(&residual).sub(&next_state).wrap();
        let solver = NewtonSolverSymbolic::new_root_solver(
            &residual,
            &next_state,
            &registry,
            solver_options,
        )?;

        Ok(HermiteSimpson {
            registry,
            solver,
            _phantom_data: PhantomData,
        })
    }
}

impl<D: SymbolicDynamics + Labelizable> Discretizer<D> for HermiteSimpson<D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let (new_state, _status, _mus, _lambdas) =
            step_intrinsic(model, state, input, dt, &self.solver, &self.registry)?;
        Ok(D::State::from_vec(new_state))
    }
}
