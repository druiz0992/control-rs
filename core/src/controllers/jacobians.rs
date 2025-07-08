use std::cmp::min;

use nalgebra::DMatrix;

use crate::controllers::ControllerOptions;
use crate::physics::ModelError;
use crate::physics::discretizer::NumericDiscretizer;
use crate::physics::models::Dynamics;
use crate::physics::traits::{Discretizer, PhysicsSim};
use crate::utils::Labelizable;
use crate::utils::evaluable::EvaluableMatrixFn;

type LinearDynamics = (DMatrix<f64>, DMatrix<f64>);
type LinearDynamicsVec = (Vec<DMatrix<f64>>, Vec<DMatrix<f64>>);

pub struct JacobianFns {
    jacobian_x_fn: EvaluableMatrixFn,
    jacobian_u_fn: EvaluableMatrixFn,
}

impl JacobianFns {
    pub fn from_sim<S>(sim: &S) -> Self
    where
        S: PhysicsSim,
        S::Model: Dynamics + Labelizable,
        S::Discretizer: NumericDiscretizer<S::Model>,
    {
        let jacobian_x_fn = sim.discretizer().jacobian_x();
        let jacobian_u_fn = sim.discretizer().jacobian_u();
        Self {
            jacobian_u_fn,
            jacobian_x_fn,
        }
    }

    pub fn linearize_step<S>(
        &self,
        sim: &S,
        idx: usize,
        general_options: &ControllerOptions<S>,
    ) -> Result<LinearDynamics, ModelError>
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
        let a_mat = self.jacobian_x_fn.evaluate(&vals)?;
        let b_mat = self.jacobian_u_fn.evaluate(&vals)?;

        Ok((a_mat, b_mat))
    }

    pub fn linearize_full<S>(
        &self,
        sim: &S,
        n_steps: usize,
        general_options: &ControllerOptions<S>,
    ) -> Result<LinearDynamicsVec, ModelError>
    where
        S: PhysicsSim,
        S::Model: Dynamics + Labelizable,
        S::Discretizer: Discretizer<S::Model>,
    {
        let n_op = min(general_options.get_u_operating().len(), n_steps - 1);
        let mut a_mat: Vec<DMatrix<f64>> = Vec::with_capacity(n_op);
        let mut b_mat: Vec<DMatrix<f64>> = Vec::with_capacity(n_op);
        let dt = general_options.get_dt();

        let labels = S::Model::labels();
        let real_params = sim.model().vectorize(labels);
        let model_params = if let Some(estimated_params) = general_options.get_estimated_params() {
            estimated_params
        } else {
            real_params.as_slice()
        };

        for k in 0..n_op {
            let mut vals = general_options.concatenate_operating_point(k)?;
            vals.extend_from_slice(model_params);
            vals.extend_from_slice(&[dt]);
            a_mat.push(self.jacobian_x_fn.evaluate(&vals)?);
            b_mat.push(self.jacobian_u_fn.evaluate(&vals)?);
        }

        Ok((a_mat, b_mat))
    }
}

impl JacobianFns {
    pub fn new(jacobian_x_fn: EvaluableMatrixFn, jacobian_u_fn: EvaluableMatrixFn) -> Self {
        Self {
            jacobian_u_fn,
            jacobian_x_fn,
        }
    }
}
