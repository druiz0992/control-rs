use crate::numeric_services::symbolic::ExprScalar;
use crate::physics::traits::SymbolicDynamics;

pub struct IndirectShooting<D: SymbolicDynamics> {
    terminal_cost: ExprScalar,
    running_cost: ExprScalar,

    u_traj: Vec<f64>,

    initial_state: D::State,
    final_state: Option<D::State>,

    time_horizon: f64,
    dt: f64,

    model: D,
}

#[derive(Clone, Default, Debug)]
pub struct IndirectShootingBuilder<D: SymbolicDynamics> {
    terminal_cost: Option<ExprScalar>,
    running_cost: Option<ExprScalar>,
    initial_state: Option<D::State>,
    final_state: Option<D::State>,
    time_horizin: Option<f64>,
    dt: Option<f64>,
    model: D,
}


impl<D:SymbolicDynamics> IndirectShootingBuilder<D>  {
    pub fn new(model:D) -> Self {
        Self {
            terminal_cost: None,
            running_cost: None,
            initial_state: None,
            final_state: None,
            time_horizin: None,
            dt: None,
            model
        }
    }

    pub fn set_terminal_cost(self, terminal_cost: ExprScalar) -> Self {
        let mut b = self;
        b.terminal_cost = Some(terminal_cost);
        b
    }
}
