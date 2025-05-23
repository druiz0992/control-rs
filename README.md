# control-rs

A modular and extensible Rust library for **optimal control**, supporting discretization, system modeling, optimization, and automatic differentiation.

## ✨ Features

- 🔧 **Discretization Methods**
  - Forward Euler
  - Backward Euler
  - Midpoint
  - Implicit Midpoint
  - Runge-Kutta 4 (RK4)
  - Hermite-Simpson

- 🧩 **Built-in Dynamical Models**
  - Double Integrator
  - Cart-Pole
  - Double Pendulum
  - (More to come...)

- 🧮 **Optimization Engine**
  - Formulate and solve trajectory optimization problems
  - Time-varying control and state constraints
  - Cost functions and bounds support

- 🧠 **Automatic Differentiation**
  - Compute Jacobians and Hessians
  - Symbolic and/or numerical differentiation
  - Works with custom user-defined dynamics and cost functions

## 🚀 Getting Started

Add to your `Cargo.toml`:

```toml
[dependencies]
optimal_control = "0.1"
```

## Usage

Here's a basic example of using `control-rs`:

```rust
use control_rs::numeric_services::symbolic::fasteval::ExprRegistry;
use control_rs::physics::discretizer::BackwardEuler;
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::simulator::{BasicSim, PhysicsSim};
use control_rs::{plotter, utils};
use std::f64::consts::PI;
use std::sync::Arc;

fn main() {
    let m1 = 1.0;
    let m2 = 1.0;
    let l1 = 1.0;
    let l2 = 1.0;

    let theta1 = PI / 1.6;
    let omega1 = 0.0;
    let theta2 = PI / 1.8;
    let omega2 = 0.0;

    let registry = Arc::new(ExprRegistry::new());
    let state0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);

    let dt = 0.01;
    let steps = 1000;

    let model = DoublePendulum::new(m1, m2, l1, l2, Some(&registry));
    let integrator = BackwardEuler::new(&model, Arc::clone(&registry)).unwrap();
    let mut sim = BasicSim::new(model, integrator, state0);

    let history = sim.rollout(dt, steps);

    let (times, states, energies) = utils::unzip3(history);

    plotter::plot_states(&times, &states, "/tmp/plot1.png").unwrap();
    plotter::plot_energy(&times, &energies, "/tmp/plot2.png").unwrap();

    plotter::display("/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}

```

## License
This project is licensed under the [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).