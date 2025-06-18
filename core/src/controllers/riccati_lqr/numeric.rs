use super::common::RiccatiRecursionGeneric;
use super::options::RiccatiLQROptions;
use crate::controllers::{Controller, ControllerState, CostFn, TrajectoryHistory};
use crate::physics::ModelError;
use crate::physics::discretizer::NumericDiscretizer;
use crate::physics::models::Dynamics;
use crate::physics::traits::PhysicsSim;
use crate::utils::Labelizable;

pub struct RiccatiRecursionNumeric<S: PhysicsSim>(RiccatiRecursionGeneric<S>);

impl<S> RiccatiRecursionNumeric<S>
where
    S: PhysicsSim + 'static,
    S::Model: Dynamics + Labelizable + 'static,
    S::Discretizer: NumericDiscretizer<S::Model> + 'static,
{
    pub fn new(
        sim: S,
        cost_fn: CostFn<S>,
        options: Option<RiccatiLQROptions<S>>,
    ) -> Result<Self, ModelError> {
        let options = options.unwrap_or_default();

        let jacobian_x_fn = sim.discretizer().to_numeric_df_dx();
        let jacobian_u_fn = sim.discretizer().to_numeric_df_du();

        let controller =
            RiccatiRecursionGeneric::new(sim, cost_fn, jacobian_x_fn, jacobian_u_fn, options)?;

        Ok(RiccatiRecursionNumeric(controller))
    }
}

impl<S> Controller<S> for RiccatiRecursionNumeric<S>
where
    S: PhysicsSim,
    S::Model: Dynamics + Labelizable,
    S::Discretizer: NumericDiscretizer<S::Model>,
{
    fn solve(
        &mut self,
        initial_state: &ControllerState<S>,
    ) -> Result<TrajectoryHistory<S>, ModelError> {
        self.0.solve(initial_state)
    }
}
#[cfg(test)]
mod tests {
    use crate::{
        cost::GenericCost,
        physics::{
            discretizer::{NumericFunction, rk4_numeric::RK4Numeric},
            models::{
                DoublePendulum, DoublePendulumState,
                double_pendulum::numeric_dynamics::{eval_dfdu, eval_dfdx},
            },
            simulator::BasicSim,
        },
    };
    use nalgebra::DMatrix;
    use std::{f64::consts::PI, sync::Arc};

    use super::*;

    #[test]
    fn test_new_riccati_recursion_numeric_ok() {
        // Pendulum parameters
        let m1 = 1.0;
        let m2 = 1.0;
        let l1 = 1.0;
        let l2 = 1.0;
        let air_resistance_coeff = 0.0;

        // Initial state
        let theta1 = PI / 1.6;
        let omega1 = 0.0;
        let theta2 = PI / 1.8;
        let omega2 = 0.0;
        let dt = 0.01;

        // Create model and wrap in Arc
        let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, None, false);
        let state_0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);

        // Closures for continuous dynamics jacobians
        let dfdx = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdx(vals)));
        let dfdu = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdu(vals)));

        // Create numeric RK4 discretizer
        let discretizer = RK4Numeric::new(Arc::new(model.clone()), dfdx, dfdu, dt).unwrap();

        // Create simulation
        let sim = BasicSim::new(model, discretizer, None);

        // Cost matrices
        let q_matrix = DMatrix::<f64>::identity(4, 4) * 1.0;
        let qn_matrix = DMatrix::<f64>::identity(4, 4) * 1.0;
        let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

        // Create cost function boxed trait object
        let cost = GenericCost::new(q_matrix, qn_matrix, r_matrix, None).unwrap();
        let cost_fn: CostFn<BasicSim<DoublePendulum, RK4Numeric<DoublePendulum>>> = Box::new(cost);

        // Create Riccati controller
        let mut controller = RiccatiRecursionNumeric::new(sim, cost_fn, None).unwrap();
        let a = controller.solve(&state_0).unwrap();
        dbg!(a);
    }
}
