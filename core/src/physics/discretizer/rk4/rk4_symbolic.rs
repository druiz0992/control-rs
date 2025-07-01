use std::marker::PhantomData;
use std::sync::Arc;

use crate::physics::discretizer::{CodeGenerator, SymbolicDiscretizer};
use crate::physics::models::state::SymbolicResult;
use crate::physics::traits::{Discretizer, State};
use crate::physics::{ModelError, constants as c, traits::SymbolicDynamics};
use crate::utils::evaluable::EvaluableMatrixFn;
use crate::utils::{Identifiable, Labelizable};
use symbolic_services::symbolic::{
    ExprRegistry, ExprScalar, ExprVector, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};

pub struct RK4Symbolic<D: SymbolicDynamics> {
    step_func: SymbolicFunction,
    registry: Arc<ExprRegistry>,
    dynamics: ExprVector,
    _phantom_data: PhantomData<D>,
}

impl<D: SymbolicDynamics> RK4Symbolic<D> {
    pub fn new(model: &D, registry: Arc<ExprRegistry>) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }

        let state = registry.get_vector(c::STATE_SYMBOLIC)?;

        let dt = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let dt2 = dt.scalef(0.5).wrap();
        let dt6 = dt.scalef(1.0 / 6.0).wrap();

        let k1_s = model.dynamics_symbolic(&state, &registry).wrap();
        let k2_s = model
            .dynamics_symbolic(&k1_s.scale(&dt2).add(&state).wrap(), &registry)
            .wrap();
        let k3_s = model
            .dynamics_symbolic(&k2_s.scale(&dt2).add(&state).wrap(), &registry)
            .wrap();
        let k4_s = model
            .dynamics_symbolic(&k3_s.scale(&dt).add(&state).wrap(), &registry)
            .wrap();
        let mut dynamics = k1_s
            .add(&k4_s)
            .add(&k2_s.scalef(2.0))
            .add(&k3_s.scalef(2.0))
            .wrap();
        dynamics = dynamics.scale(&dt6);
        dynamics = dynamics.add(&state).wrap();
        let step_func = dynamics.to_fn(&registry)?;

        // vars
        let mut vars = state.to_vec();
        let input = registry.get_vector(c::INPUT_SYMBOLIC)?;
        vars.extend_from_slice(&input.to_vec());
        let model_params = registry.get_vector(c::MODEL_SYMBOLIC)?;
        vars.extend_from_slice(&model_params.to_vec());
        vars.extend_from_slice(&[dt]);

        let step_func = SymbolicFunction::new(step_func, &vars);

        Ok(Self {
            step_func,
            registry,
            dynamics,
            _phantom_data: PhantomData,
        })
    }
}

impl<D: SymbolicDynamics + Labelizable> Discretizer<D> for RK4Symbolic<D> {
    fn step(
        &self,
        model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        let mut vals = state.to_vec();
        if let Some(u) = input {
            vals.extend_from_slice(&u.to_vec());
        } else {
            vals.extend_from_slice(&D::Input::default().to_vec())
        }
        vals.extend_from_slice(&model.vectorize(D::labels()));
        vals.extend_from_slice(&[dt]);

        Ok(SymbolicResult::new(self.step_func.eval(&vals)).try_into_eval_result()?)
    }
}

impl<D: SymbolicDynamics + Labelizable> SymbolicDiscretizer<D> for RK4Symbolic<D> {
    fn jacobian_x(&self) -> Result<EvaluableMatrixFn, ModelError> {
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let params_symbol = self.registry.get_vector(c::MODEL_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol).extend(&params_symbol);

        let jacobian_x = self
            .dynamics
            .jacobian(&state_symbol)?
            .to_fn(&self.registry)?;
        Ok(Box::new(SymbolicFunction::new(
            jacobian_x,
            &jacobian_symbols,
        )))
    }

    fn jacobian_u(&self) -> Result<EvaluableMatrixFn, ModelError> {
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let params_symbol = self.registry.get_vector(c::MODEL_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol.extend(&input_symbol).extend(&params_symbol);

        let jacobian_u = self
            .dynamics
            .jacobian(&input_symbol)?
            .to_fn(&self.registry)?;
        Ok(Box::new(SymbolicFunction::new(
            jacobian_u,
            &jacobian_symbols,
        )))
    }
}

impl<D: SymbolicDynamics + Labelizable + Identifiable> CodeGenerator<D> for RK4Symbolic<D> {
    fn to_numeric_jacobian_x(&self) -> Result<(), ModelError> {
        let params = ExprVector::new(D::labels());
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol
            .extend(&input_symbol)
            .extend(&params)
            .extend(&ExprVector::new(&[c::TIME_DELTA_SYMBOLIC]));
        let mod_name = &format!("rk4/{}", D::name());
        let func_name = &format!("rk4_{}_jacobian_x", D::name());

        let jacobian_x_expr = self.dynamics.jacobian(&state_symbol)?;
        jacobian_x_expr.rustify(&jacobian_symbols, func_name, mod_name)?;

        Ok(())
    }

    fn to_numeric_jacobian_u(&self) -> Result<(), ModelError> {
        let params = ExprVector::new(D::labels());
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol
            .extend(&input_symbol)
            .extend(&params)
            .extend(&ExprVector::new(&[c::TIME_DELTA_SYMBOLIC]));
        let mod_name = &format!("rk4/{}", D::name());
        let func_name = &format!("rk4_{}_jacobian_u", D::name());

        let jacobian_u_expr = self.dynamics.jacobian(&input_symbol)?;
        jacobian_u_expr.rustify(&jacobian_symbols, func_name, mod_name)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::models::{DoublePendulum, DoublePendulumState};

    #[test]
    fn test_rk4_symbolic_step() {
        let registry = Arc::new(ExprRegistry::new());
        let dynamics = DoublePendulum::new(1.0, 2.0, 1.5, 2.5, 0.0, Some(&registry));

        let theta1 = 0.0;
        let omega1 = 0.0;
        let theta2 = 0.0;
        let omega2 = 0.0;

        let state = DoublePendulumState::new(theta1, omega1, theta2, omega2);
        let rk4_symbolic = RK4Symbolic::new(&dynamics, Arc::clone(&registry)).unwrap();
        let dt = 0.1;

        let next_state = rk4_symbolic.step(&dynamics, &state, None, dt).unwrap();

        assert!((next_state.omega1 - 0.0).abs() < 1e-6);
    }
}
