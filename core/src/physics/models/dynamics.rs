use crate::numeric_services::symbolic::{
    ExprRegistry, ExprScalar, ExprVector, SymbolicExpr, SymbolicFunction,
};
use crate::physics::constants as c;
use crate::physics::traits::State;
use crate::physics::{Energy, ModelError};
use crate::utils::evaluable::Evaluable;
use nalgebra::{DMatrix, Vector2};
use std::sync::Arc;

pub trait Dynamics: Clone {
    type State: Clone + super::state::State;
    type Input: Clone + super::state::State;

    fn dynamics(&self, state: &Self::State, input: Option<&Self::Input>) -> Self::State;
    fn energy(&self, _state: &Self::State) -> Option<Energy> {
        None
    }
    fn state_dims(&self) -> (usize, usize);
}

pub trait SymbolicDynamics: Dynamics {
    fn dynamics_symbolic(&self, state: &ExprVector, registry: &Arc<ExprRegistry>) -> ExprVector;
    fn cost_linear_term(
        &self,
        _dt: &ExprScalar,
        _registry: &Arc<ExprRegistry>,
    ) -> Option<ExprVector> {
        None
    }
    fn linearize(
        &self,
        x_ref: &[f64],
        u_ref: &[f64],
        registry: &Arc<ExprRegistry>,
    ) -> Result<(DMatrix<f64>, DMatrix<f64>), ModelError> {
        let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol);
        let mut vals = x_ref.to_vec();
        vals.extend(u_ref.to_vec());

        let jacobian_x = self
            .dynamics_symbolic(&state_symbol, registry)
            .jacobian(&state_symbol)?
            .to_fn(registry)?;
        let df_dx = SymbolicFunction::new(jacobian_x, &jacobian_symbols);
        let a_mat: DMatrix<f64> = df_dx.evaluate(&vals)?;

        let jacobian_u = self
            .dynamics_symbolic(&state_symbol, registry)
            .jacobian(&input_symbol)?
            .to_fn(registry)?;
        let df_du = SymbolicFunction::new(jacobian_u, &jacobian_symbols);
        let b_mat: DMatrix<f64> = df_du.evaluate(&vals)?;

        Ok((a_mat, b_mat))
    }
}

pub trait LinearDynamics: Dynamics {
    fn get_state_slice(&self) -> &DMatrix<f64>;
    fn get_control_slice(&self) -> &DMatrix<f64>;
}

pub trait Renderable: Dynamics {
    fn render_joints(&self, state: &Self::State, screen_dims: (f32, f32)) -> Vec<Vector2<f32>>;
}
