use crate::physics::traits::{Discretizer, Dynamics, PhysicsSim};
use crate::physics::{Energy, ModelError};

pub struct BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    discretizer: D,
    model: M,
}

impl<M, D> BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    pub fn new(model: M, discretizer: D) -> Self {
        BasicSim { discretizer, model }
    }
}

impl<M, D> PhysicsSim for BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    type Model = M;

    fn rollout(
        &self,
        initial_state: &M::State,
        dt: f64,
        steps: usize,
    ) -> Vec<(f64, M::State, Energy)> {
        let mut history = Vec::with_capacity(steps);
        let mut t = 0.0;
        let state = initial_state;
        for _ in 0..steps {
            let energy = self.model.energy(state).unwrap_or_default();

            history.push((t, state.clone(), energy));
            let _ = self.step(state, None, dt);
            t += dt;
        }
        history
    }

    fn step(
        &self,
        state: &M::State,
        input: Option<&[f64]>,
        dt: f64,
    ) -> Result<M::State, ModelError> {
        let state = self.discretizer.step(&self.model, state, input, dt)?;
        Ok(state)
    }

    fn model(&self) -> &Self::Model {
        &self.model
    }
}
