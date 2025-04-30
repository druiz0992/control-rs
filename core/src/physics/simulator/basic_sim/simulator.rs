use crate::physics::traits::{Discretizer, Dynamics, PhysicsSim};
use crate::physics::{Energy, ModelError};

pub struct BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    discretizer: D,
    model: M,
    state: M::State,
}

impl<M, D> BasicSim<M, D>
where
    M: Dynamics,
    D: Discretizer<M>,
{
    pub fn new(model: M, discretizer: D, initial_state: M::State) -> Self {
        BasicSim {
            discretizer,
            model,
            state: initial_state,
        }
    }

    pub fn energy(&self, state: &M::State) -> Energy {
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
            let energy = self.energy(&self.state);
            history.push((t, self.state.clone(), energy));
            let _ = self.step(dt);
            t += dt;
        }
        history
    }

    fn step(&mut self, dt: f64) -> Result<&M::State, ModelError> {
        self.state = self.discretizer.step(&self.model, &self.state, dt)?;
        Ok(&self.state)
    }

    fn model(&self) -> &Self::Model {
        &self.model
    }

    fn state(&self) -> &<Self::Model as Dynamics>::State {
        &self.state
    }
}
