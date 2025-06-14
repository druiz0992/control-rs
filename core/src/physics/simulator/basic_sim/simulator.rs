use std::sync::Arc;

use crate::numeric_services::symbolic::ExprRegistry;
use crate::physics::ModelError;
use crate::physics::traits::{Discretizer, Dynamics, PhysicsSim};

#[derive(Debug, Clone)]
pub struct BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    discretizer: D,
    model: M,
    registry: Option<Arc<ExprRegistry>>,
}

impl<M, D> BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    pub fn new(model: M, discretizer: D, registry: Option<Arc<ExprRegistry>>) -> Self {
        BasicSim {
            discretizer,
            model,
            registry,
        }
    }
}

impl<M, D> PhysicsSim for BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    type Model = M;
    type Discretizer = D;

    fn rollout(
        &self,
        initial_state: &M::State,
        input: Option<&Vec<M::Input>>,
        dt: f64,
        steps: usize,
    ) -> Result<Vec<M::State>, ModelError> {
        let mut history = Vec::with_capacity(steps - 1);
        let mut state = initial_state.clone();
        history.push(state.clone());
        for i in 0..steps - 1 {
            let current_input = input.and_then(|inputs| inputs.get(i));

            state = self.step(&state, current_input, dt)?;
            history.push(state.clone());
        }
        Ok(history)
    }

    fn step(
        &self,
        state: &M::State,
        input: Option<&M::Input>,
        dt: f64,
    ) -> Result<M::State, ModelError> {
        let state = self.discretizer.step(&self.model, state, input, dt)?;
        Ok(state)
    }

    fn model(&self) -> &Self::Model {
        &self.model
    }
    fn discretizer(&self) -> &Self::Discretizer {
        &self.discretizer
    }

    fn update_model(&mut self, params: &[f64]) -> Result<(), ModelError> {
        self.model.update(params, self.registry.as_ref())
    }
}
