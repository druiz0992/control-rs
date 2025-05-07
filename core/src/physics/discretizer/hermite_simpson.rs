use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics, State};
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

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

pub struct HermiteSimpson {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolver,
}

impl HermiteSimpson {
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let dt_expr = ExprScalar::new("dt");
        let dt6 = dt_expr.scalef(1.0 / 6.0);
        let dt8 = dt_expr.scalef(1.0 / 8.0);
        let current_state = registry.get_vector("state")?;
        let next_state = current_state.build_next();
        registry.insert_vector("next_state", next_state.clone());

        let dyn_current_state = model
            .dynamics_symbolic(current_state.clone().wrap(), &registry)
            .wrap();

        let dyn_next_state = model
            .dynamics_symbolic(next_state.clone().wrap(), &registry)
            .wrap();

        let mid_state = dyn_current_state
            .sub(&dyn_next_state)
            .wrap()
            .scale(&dt8)
            .add(&current_state.add(&next_state).wrap().scalef(0.5));

        let dyn_mid_state = model.dynamics_symbolic(mid_state.wrap(), &registry).wrap();

        let mut residual = dyn_current_state
            .add(&dyn_mid_state.scalef(4.0))
            .add(&dyn_next_state)
            .wrap()
            .scale(&dt6);
        residual = current_state.add(&residual).sub(&next_state).wrap();
        let solver = NewtonSolver::new_root_solver(&residual, &next_state, &registry, None)?;

        Ok(HermiteSimpson { registry, solver })
    }
}

impl<D> Discretizer<D> for HermiteSimpson
where
    D: Dynamics,
{
    fn step(&mut self, _model: &D, state: &D::State, dt: f64) -> Result<D::State, ModelError> {
        self.registry.insert_var("dt", dt);
        self.registry.insert_vec_as_vars("state", &state.as_vec())?;
        self.registry
            .insert_vec_as_vars("next_state", &state.as_vec())?;

        let history = self.solver.solve(&state.as_vec(), &self.registry)?;
        Ok(D::State::from_vec(history.last().cloned().ok_or_else(
            || ModelError::SolverError(String::from("Solver failed")),
        )?))
    }
}

impl Describable for HermiteSimpson {
    fn name(&self) -> &'static str {
        "Hermite-Simpson"
    }
}
