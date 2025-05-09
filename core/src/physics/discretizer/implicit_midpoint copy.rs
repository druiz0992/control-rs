use crate::numeric_services::symbolic::{ExprRegistry, ExprScalar};
use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, Dynamics, State};
use crate::solver::newton::NewtonSolver;
use std::sync::Arc;

// residual(x_next) = x_next - x_k - dt * f((x_k + x_next) / 2)

pub struct ImplicitMidpoint {
    registry: Arc<ExprRegistry>,
    solver: NewtonSolver,
}

impl ImplicitMidpoint {
    pub fn new<D: Dynamics>(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let dt_expr = ExprScalar::new("dt");
        let current_state = registry.get_vector("state")?;
        let next_state = current_state.build_next();
        registry.insert_vector("next_state", next_state.clone());

        let mid_state = current_state.add(&next_state).wrap().scalef(0.5).wrap();

        let dyn_mid_state = model
            .dynamics_symbolic(&mid_state,  &registry)
            .wrap()
            .scale(&dt_expr);

        let residual = next_state.sub(&current_state).sub(&dyn_mid_state).wrap();
        let solver = NewtonSolver::new_root_solver(&residual, &next_state, &registry, None)?;

        Ok(ImplicitMidpoint { registry, solver })
    }
}

impl<D> Discretizer<D> for ImplicitMidpoint
where
    D: Dynamics,
{
    fn step(
        &mut self,
        model: &D,
        state: &D::State,
        _input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        self.registry.insert_var("dt", dt);
        self.registry.insert_vec_as_vars("state", &state.as_vec())?;
        self.registry
            .insert_vec_as_vars("next_state", &state.as_vec())?;

        // pass state (vk+1) so that it can find qk+1
        let history = self.solver.solve(&state.as_vec(), &self.registry)?;
        Ok(D::State::from_vec(history.last().cloned().ok_or_else(
            || ModelError::SolverError(String::from("Solver failed")),
        )?))
    }
}

impl Describable for ImplicitMidpoint {
    fn name(&self) -> &'static str {
        "Implicit-Midpoint"
    }
}
