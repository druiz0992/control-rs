use std::marker::PhantomData;

use crate::physics::traits::{Dynamics, PhysicsSim, Renderable};

use super::Animation;
use async_trait::async_trait;
use macroquad::prelude::*;

#[derive(Clone, Debug, Default)]
pub struct Macroquad<S: PhysicsSim> {
    _phantom: std::marker::PhantomData<S>,
}

impl<S> Macroquad<S>
where
    S: PhysicsSim,
{
    pub fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

#[async_trait]
impl<S> Animation for Macroquad<S>
where
    S: PhysicsSim + Send,
    S::Model: Renderable<State = <S::Model as Dynamics>::State>,
{
    type Simulator = S;

    async fn run_animation(self, simulator: &mut Self::Simulator, screen_dims: (f32, f32)) {
        loop {
            clear_background(BLACK);
            let dt = get_frame_time();
            simulator.step(dt as f64);

            let model = simulator.model();
            let state = simulator.state();
            let joints = model.render_joints(state, screen_dims);

            for win in joints.windows(2) {
                draw_line(win[0].x, win[0].y, win[1].x, win[1].y, 2.0, WHITE);
            }

            for p in &joints[1..] {
                draw_circle(p.x, p.y, 6.0, RED);
            }

            next_frame().await;
        }
    }
}
