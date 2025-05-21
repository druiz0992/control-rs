use crate::physics::traits::{Discretizer, Dynamics, PhysicsSim};
use crate::physics::{Energy, ModelError};

pub struct BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    discretizer: D,
    state: M::State,
    model: M,
}

impl<M, D> BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    pub fn new(model: M, discretizer: D, initial_state: M::State) -> Self {
        BasicSim {
            discretizer,
            state: initial_state,
            model,
        }
    }

    pub fn energy(&self, state: &M::State) -> Option<Energy> {
        self.model.energy(state)
    }
}

impl<M, D> PhysicsSim for BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    type Model = M;

    fn simulate_steps(&mut self, dt: f64, steps: usize) -> Vec<(f64, M::State, Energy)> {
        let mut history = Vec::with_capacity(steps);
        let mut t = 0.0;
        for _ in 0..steps {
            let energy = self.energy(&self.state).unwrap_or_default();

            history.push((t, self.state.clone(), energy));
            let _ = self.step(None, dt);
            t += dt;
        }
        history
    }

    fn step(&mut self, input: Option<&[f64]>, dt: f64) -> Result<&M::State, ModelError> {
        self.state = self.discretizer.step(&self.model, &self.state, input, dt)?;
        Ok(&self.state)
    }

    fn model(&self) -> &Self::Model {
        &self.model
    }

    fn state(&self) -> &<Self::Model as Dynamics>::State {
        &self.state
    }
}
