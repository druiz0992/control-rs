use std::marker::PhantomData;
use std::sync::Arc;

use crate::physics::discretizer::utils::{extend_vals, extend_vars};
use crate::physics::discretizer::{CodeGenerator, SymbolicDiscretizer};
use crate::physics::models::state::SymbolicResult;
use crate::physics::traits::Discretizer;
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

        let vars = extend_vars(&registry)?;
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
        let vals = extend_vals(model, state, input, dt);

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

        println!("Computing df_dx...");
        let jacobian_x_expr = self.dynamics.jacobian(&state_symbol)?;
        println!("Rustifying df_dx...");
        jacobian_x_expr.rustify(&jacobian_symbols, func_name, mod_name, true)?;

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

        println!("Computing df_du...");
        let jacobian_u_expr = self.dynamics.jacobian(&input_symbol)?;
        println!("Rustifying df_du...");
        jacobian_u_expr.rustify(&jacobian_symbols, func_name, mod_name, true)?;

        Ok(())
    }

    fn to_numeric_da(&self) -> Result<(), ModelError> {
        let params = ExprVector::new(D::labels());
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol
            .extend(&input_symbol)
            .extend(&params)
            .extend(&ExprVector::new(&[c::TIME_DELTA_SYMBOLIC]));
        let mod_name = &format!("rk4/{}", D::name());
        let d2f_dxx = &format!("rk4_{}_d2f_dxx", D::name());
        let d2f_dxu = &format!("rk4_{}_d2f_dxu", D::name());

        println!("Computing d2f_dxx and d2f_dxu...");
        let db = self
            .dynamics
            .hessian(&state_symbol, &[state_symbol.clone(), input_symbol])?;
        println!("Rustifying d2f_dxx...");
        db[0].rustify(&jacobian_symbols, d2f_dxx, mod_name, true)?;
        println!("Rustifying d2f_dxu...");
        db[1].rustify(&jacobian_symbols, d2f_dxu, mod_name, true)?;

        Ok(())
    }
    fn to_numeric_db(&self) -> Result<(), ModelError> {
        let params = ExprVector::new(D::labels());
        let state_symbol = self.registry.get_vector(c::STATE_SYMBOLIC).unwrap();
        let input_symbol = self.registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
        let jacobian_symbols = state_symbol
            .extend(&input_symbol)
            .extend(&params)
            .extend(&ExprVector::new(&[c::TIME_DELTA_SYMBOLIC]));
        let mod_name = &format!("rk4/{}", D::name());
        let d2f_dux = &format!("rk4_{}_d2f_dux", D::name());
        let d2f_duu = &format!("rk4_{}_d2f_duu", D::name());

        println!("Computing d2f_duu and d2f_dux...");
        let db = self
            .dynamics
            .hessian(&input_symbol, &[input_symbol.clone(), state_symbol])?;
        println!("Rustifying d2f_duu...");
        db[0].rustify(&jacobian_symbols, d2f_duu, mod_name, true)?;
        println!("Rustifying d2f_dux...");
        db[1].rustify(&jacobian_symbols, d2f_dux, mod_name, true)?;

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
