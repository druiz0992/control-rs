use std::marker::PhantomData;

use nalgebra::DMatrix;

use crate::physics::ModelError;
use crate::physics::traits::{Discretizer, LinearDynamics, State};

use super::LinearDiscretizer;

const EXPM_TOL: f64 = 1e-6;
const EXPM_MAX_ITERATIONS: usize = 50;

#[derive(Default, Clone)]
pub struct ZOH<D>
where
    D: LinearDynamics,
{
    state_matrix_d: DMatrix<f64>,
    control_matrix_d: DMatrix<f64>,
    dt: f64,
    _phantom_data: PhantomData<D>,
}

impl<D> ZOH<D>
where
    D: LinearDynamics,
{
    pub fn new(model: &D, dt: f64) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        let (state_matrix_d, control_matrix_d) = discretize_a_b_matrices(model, dt);

        Ok(Self {
            dt,
            state_matrix_d,
            control_matrix_d,
            _phantom_data: PhantomData,
        })
    }

    pub fn get_dt(&self) -> f64 {
        self.dt
    }
}

impl<D> Discretizer<D> for ZOH<D>
where
    D: LinearDynamics,
{
    fn step(
        &self,
        _model: &D,
        state: &D::State,
        input: Option<&D::Input>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        if dt != self.dt {
            return Err(ModelError::DiscretizerError(format!(
                "Zero Order Hold discretizer expects a dt with value {}",
                self.dt
            )));
        }
        let sm = &self.state_matrix_d;
        let cm = &self.control_matrix_d;
        let dv_state = state.to_vector();
        let dv_input = input.unwrap_or(&D::Input::default()).to_vector();

        let r = sm * dv_state + cm * dv_input;

        Ok(D::State::from_slice(r.as_slice()))
    }
}

fn discretize_a_b_matrices<D>(model: &D, dt: f64) -> (DMatrix<f64>, DMatrix<f64>)
where
    D: LinearDynamics,
{
    let state_matrix = model.get_state_slice();
    let control_matrix = model.get_control_slice();
    let n = state_matrix.nrows();
    let m = control_matrix.ncols();

    let augmented_matrix = build_augmented_matrix(state_matrix, control_matrix);
    // Construct augmented matrix M of size (n+m) x (n+m)
    let exp_m = matrix_exponential(&(&augmented_matrix * dt)); // your expm function

    // Extract A_d and B_d
    let state_matrix_d = exp_m.view((0, 0), (n, n)).into_owned();
    let control_matrix_d = exp_m.view((0, n), (n, m)).into_owned();

    (state_matrix_d, control_matrix_d)
}

fn build_augmented_matrix(mat_a: &DMatrix<f64>, mat_b: &DMatrix<f64>) -> DMatrix<f64> {
    let n = mat_a.nrows();
    let m = mat_b.ncols();
    let mut augmented_m = DMatrix::<f64>::zeros(n + m, n + m);

    augmented_m.view_mut((0, 0), (n, n)).copy_from(mat_a);
    augmented_m.view_mut((0, n), (n, m)).copy_from(mat_b);

    augmented_m
}

fn matrix_exponential(mat: &DMatrix<f64>) -> DMatrix<f64> {
    let n = mat.nrows();
    let mut result = DMatrix::<f64>::identity(n, n);
    let mut term = DMatrix::<f64>::identity(n, n);
    let mut factorial = 1.0;

    for i in 1..EXPM_MAX_ITERATIONS {
        term = mat * &term;
        factorial *= i as f64;
        let delta = &term / factorial;
        result += &delta;
        if delta.norm() < EXPM_TOL {
            break;
        }
    }

    result
}

impl<D: LinearDynamics> LinearDiscretizer<D> for ZOH<D> {
    fn jacobian_x(&self) -> &DMatrix<f64> {
        &self.state_matrix_d
    }
    fn jacobian_u(&self) -> &DMatrix<f64> {
        &self.control_matrix_d
    }
}
