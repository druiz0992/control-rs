use control_rs::controllers::Controller;
use control_rs::controllers::QPLQR;
use control_rs::cost::generic::GenericCost;
use control_rs::physics::discretizer::ZOH;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::plotter;
use nalgebra::{DMatrix, dmatrix};

fn main() {
    let state_matrix = dmatrix![0.0,1.0; 0.0,0.0];
    let control_matrix = dmatrix![0.0; 1.0];
    let model = LtiModel::<2, 0, 1>::new(state_matrix, control_matrix).unwrap();

    let initial_state = LtiState::<2, 0>::new([1.0, 0.0]);
    let dt = 0.1;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let integrator = ZOH::new(&model, dt).unwrap();

    let sim = BasicSim::new(model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(2, 2);
    let qn_matrix = DMatrix::<f64>::identity(2, 2);
    let r_matrix = DMatrix::<f64>::identity(1, 1) * 0.1;
    let zero_x: Vec<_> = (0..n_steps).map(|_| LtiState::default()).collect();
    let cost = GenericCost::<_, LtiInput<1, 0>>::new(q_matrix, qn_matrix, r_matrix, zero_x.clone())
        .unwrap();
    let u_limits = (-1.0, 1.0);
    let mut controller = QPLQR::new(
        sim,
        Box::new(cost.clone()),
        &initial_state,
        Some(u_limits),
        //None,
        sim_time,
        dt,
    )
    .unwrap();

    controller.solve(&initial_state).unwrap();
    let x_traj = controller.rollout(&initial_state).unwrap();
    let u_traj = controller.get_u_traj();

    let times: Vec<_> = (0..x_traj.len()).map(|i| i as f64 * dt).collect();

    plotter::plot_states(&times, &x_traj, "/tmp/plot1.png").unwrap();
    plotter::display("/tmp/plot1.png").unwrap();

    plotter::plot_states(&times, &u_traj, "/tmp/plot2.png").unwrap();
    plotter::display("/tmp/plot2.png").unwrap();
}
