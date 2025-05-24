use nalgebra::DMatrix;

use crate::{cost::CostFunction, physics::{models::Dynamics, traits::{PhysicsSim, State}, ModelError}};

pub mod indirect_shooting;


type ControllerState<S> = <<S as PhysicsSim>::Model as Dynamics>::State;
type ControllerInput<S> = <<S as PhysicsSim>::Model as Dynamics>::Input;
type CostFn<S> =
    Box<dyn CostFunction<State = ControllerState<S>, Input = ControllerInput<S>>>;

pub struct InputTrajectory<S: PhysicsSim>(Vec<ControllerInput<S>>);

impl<S: PhysicsSim> InputTrajectory<S> {
    pub fn as_slice(&self) -> &[ControllerInput<S>] {
        &self.0
    }
    pub fn as_vec(&self) -> &Vec<ControllerInput<S>> {
        &self.0
    }
    pub fn as_mut_vec(&mut self) -> &Vec<ControllerInput<S>> {
        &self.0
    }
    pub fn to_vec(&self) -> Vec<ControllerInput<S>> {
        self.0.clone()
    }
}
    
impl<S: PhysicsSim> From<&InputTrajectory<S>> for DMatrix<f64> {
    fn from(value: &InputTrajectory<S>) -> DMatrix<f64> {
        if value.0.is_empty() {
            return DMatrix::from_column_slice(0, 0, &[]);
        }
        let input_dims = value.0[0].to_vec().len();
        let mut mat = DMatrix::<f64>::zeros(input_dims, value.0.len());
        value.0.iter().enumerate().for_each(|(i, v)| {
            mat.set_column(i, &v.to_vector());
        });
        mat
    }
}

impl<S: PhysicsSim> TryFrom<&DMatrix<f64>> for InputTrajectory<S> {
    type Error = ModelError;
    fn try_from(value: &DMatrix<f64>) -> Result<Self, Self::Error> {
        if value.is_empty() {
            return Err(ModelError::ConfigError(
                "Input matrix cannot be empty".into(),
            ));
        }

        Ok(InputTrajectory(
            value
                .column_iter()
                .map(|v| ControllerInput::<S>::from_slice(v.as_slice()))
                .collect(),
        ))
    }
}