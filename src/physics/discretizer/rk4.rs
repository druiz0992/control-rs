use crate::physics::traits::{Discretizer, Dynamics};

pub struct RK4;

impl RK4 {
    pub fn new() -> Self {
        Self
    }
}

impl<D> Discretizer<D> for RK4
where
    D: Dynamics,
{
    fn step(&mut self, model: &D, state: &D::State, dt: f64) -> D::State {
        let k1 = model.dynamics(state);
        let k2 = model.dynamics(&(state.clone() + k1.clone() * (dt * 0.5)));
        let k3 = model.dynamics(&(state.clone() + k2.clone() * (dt * 0.5)));
        let k4 = model.dynamics(&(state.clone() + k3.clone() * dt));

        state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
    }
}
