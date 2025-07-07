use nalgebra::DMatrix;

use crate::controllers::ControllerOptions;
use crate::physics::ModelError;
use crate::physics::discretizer::NumericDiscretizer;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim};
use crate::utils::Labelizable;
use crate::utils::evaluable::EvaluableMatrixFn;

type LinearDynamicsSecondOrder = (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>, DMatrix<f64>);
pub struct HessianFns {
    hessian_xx_fn: EvaluableMatrixFn,
    hessian_xu_fn: EvaluableMatrixFn,
    hessian_ux_fn: EvaluableMatrixFn,
    hessian_uu_fn: EvaluableMatrixFn,
}

impl HessianFns {
    pub fn from_sim<S>(sim: &S) -> Option<Self>
    where
        S: PhysicsSim,
        S::Model: Dynamics + Labelizable,
        S::Discretizer: NumericDiscretizer<S::Model>,
    {
        let hessian_xx_fn = sim.discretizer().hessian_xx();
        let hessian_xu_fn = sim.discretizer().hessian_xu();
        let hessian_ux_fn = sim.discretizer().hessian_ux();
        let hessian_uu_fn = sim.discretizer().hessian_uu();
        if let (
            Some(hessian_xx_fn),
            Some(hessian_xu_fn),
            Some(hessian_ux_fn),
            Some(hessian_uu_fn),
        ) = (hessian_xx_fn, hessian_xu_fn, hessian_ux_fn, hessian_uu_fn)
        {
            return Some(Self {
                hessian_uu_fn,
                hessian_ux_fn,
                hessian_xu_fn,
                hessian_xx_fn,
            });
        }

        None
    }

    pub fn linearize_second_order_step<S>(
        &self,
        sim: &S,
        idx: usize,
        general_options: &ControllerOptions<S>,
    ) -> Result<LinearDynamicsSecondOrder, ModelError>
    where
        S: PhysicsSim,
        S::Model: Dynamics + Labelizable,
        S::Discretizer: Discretizer<S::Model>,
    {
        let dt = general_options.get_dt();
        let labels = S::Model::labels();
        let real_params = sim.model().vectorize(labels);
        let model_params = if let Some(estimated_params) = general_options.get_estimated_params() {
            estimated_params
        } else {
            real_params.as_slice()
        };

        let mut vals = general_options.concatenate_operating_point(idx)?;
        vals.extend_from_slice(model_params);
        vals.extend_from_slice(&[dt]);
        let a_x = self.hessian_xx_fn.evaluate(&vals)?;
        let a_u = self.hessian_xu_fn.evaluate(&vals)?;
        let b_x = self.hessian_ux_fn.evaluate(&vals)?;
        let b_u = self.hessian_uu_fn.evaluate(&vals)?;

        Ok((a_x, a_u, b_x, b_u))
    }
}

impl HessianFns {
    pub fn new(
        hessian_xx_fn: Option<EvaluableMatrixFn>,
        hessian_xu_fn: Option<EvaluableMatrixFn>,
        hessian_ux_fn: Option<EvaluableMatrixFn>,
        hessian_uu_fn: Option<EvaluableMatrixFn>,
    ) -> Option<Self> {
        {
            if let (
                Some(hessian_xx_fn),
                Some(hessian_xu_fn),
                Some(hessian_ux_fn),
                Some(hessian_uu_fn),
            ) = (hessian_xx_fn, hessian_xu_fn, hessian_ux_fn, hessian_uu_fn)
            {
                return Some(Self {
                    hessian_uu_fn,
                    hessian_ux_fn,
                    hessian_xu_fn,
                    hessian_xx_fn,
                });
            }

            None
        }
    }
}
