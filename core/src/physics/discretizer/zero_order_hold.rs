use nalgebra::{DMatrix, DVector};

use crate::physics::ModelError;
use crate::physics::traits::{Describable, Discretizer, LinearDynamics, State};

const EXPM_TOL: f64 = 1e-6;
const EXPM_MAX_ITERATIONS: usize = 50;

#[derive(Default)]
pub struct ZOH<D>
where
    D: LinearDynamics,
{
    model: D,
    state_matrix_d: DMatrix<f64>,
    control_matrix_d: DMatrix<f64>,
    dt: f64,
}

impl<D> ZOH<D>
where
    D: LinearDynamics,
{
    pub fn new(model: D, dt: f64) -> Result<Self, ModelError> {
        let (_, v_dims) = model.state_dims();
        if v_dims > 0 {
            return Err(ModelError::Unexpected("Insuported Discretizer".into()));
        }
        let (state_matrix_d, control_matrix_d) = discretize_a_b_matrices(&model, dt);

        Ok(Self {
            model,
            dt,
            state_matrix_d,
            control_matrix_d,
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
        &mut self,
        state: &D::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<D::State, ModelError> {
        if dt != self.dt {
            return Err(ModelError::DiscretizerError(format!(
                "Zero Order Hold discretizer expects a dt with value {}",
                self.dt
            )));
        }
        let sm = self.state_matrix_d.clone();
        let cm = self.control_matrix_d.clone();
        let dv_state = DVector::from_vec(state.to_vec());
        let dv_input = DVector::from_row_slice(input.unwrap_or(&vec![0.0; cm.ncols()]));

        let r = sm * dv_state + cm * dv_input;

        Ok(D::State::from_vec(r.as_slice().to_vec()))
    }

    fn get_model(&self) -> &D {
        &self.model
    }
}

impl<D> Describable for ZOH<D>
where
    D: LinearDynamics,
{
    fn name(&self) -> &'static str {
        "Zero Order Hold"
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
