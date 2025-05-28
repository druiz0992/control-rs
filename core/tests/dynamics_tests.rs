use control_rs::controllers::riccati_lqr::options::RiccatiLQROptions;
use control_rs::controllers::{Controller, QPLQR, RiccatiRecursionLQR};
use control_rs::cost::generic::GenericCost;
use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::{
    LinearDiscretizer, RK4, RK4Symbolic, SymbolicDiscretizer, ZOH,
};
use control_rs::physics::models::quadrotor_2d::input::Quadrotor2DInput;
use control_rs::physics::models::quadrotor_2d::model::Quadrotor2D;
use control_rs::physics::models::quadrotor_2d::state::Quadrotor2DState;
use control_rs::physics::models::{LtiInput, LtiModel, LtiState};
use control_rs::physics::simulator::BasicSim;
use control_rs::physics::traits::{State, SymbolicDynamics};
use control_rs::utils::evaluable::Evaluable;
use macroquad::window::InternalGlContext;
use nalgebra::DMatrix;
use std::f64::consts::PI;
use std::sync::Arc;

#[test]
fn test_linearize_quadrotor() {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let u_limits = (0.2 * m * c::GRAVITY, 0.6 * m * c::GRAVITY);
    let input_hover = LtiInput::<2, 0>::new([0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY]);

    let state_hover = LtiState::<6, 0>::new([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let state_0 = LtiState::<6, 0>::new([1.0, 2.0, 0.0, 0.0, 0.0, 0.0]);
    let state_ref = LtiState::<6, 0>::new([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);

    let registry = Arc::new(ExprRegistry::new());
    let model = Quadrotor2D::new(m, j, l, Some(&registry));

    let (a_t, b_t) = model
        .linearize(&state_hover.to_vec(), &input_hover.to_vec(), &registry)
        .unwrap();

    let lti_model = LtiModel::<6, 0, 2>::new(a_t, b_t).unwrap();
    let dt = 0.05;
    let sim_time = 10.0;
    let n_steps = (sim_time / dt) as usize + 1;

    let integrator = ZOH::new(&lti_model, dt).unwrap();
    let sim = BasicSim::new(lti_model.clone(), integrator);

    let q_matrix = DMatrix::<f64>::identity(6, 6);
    let qn_matrix = DMatrix::<f64>::identity(6, 6);
    let r_matrix = DMatrix::<f64>::identity(2, 2) * 0.01;

    let zero_x: Vec<_> = (0..n_steps).map(|_| LtiState::<6, 0>::default()).collect();

    let cost = GenericCost::<_, LtiInput<2, 0>>::new(q_matrix, qn_matrix, r_matrix, zero_x.clone())
        .unwrap();

    let mut lqr_qp = QPLQR::new(
        sim.clone(),
        Box::new(cost.clone()),
        &state_0,
        Some(u_limits),
        sim_time,
        dt,
    )
    .unwrap();
    lqr_qp.solve(&state_0).unwrap();
    let x_traj_qp = lqr_qp.rollout(&state_0);
    let u_traj_qp = lqr_qp.get_u_traj();

    let mut options = RiccatiLQROptions::enable_inifinte_horizon();
    options.set_x_ref(&state_ref);
    options.set_u_equilibrium(&input_hover);

    let mut lqr_riccati =
        RiccatiRecursionLQR::new(sim.clone(), Box::new(cost.clone()), sim_time, dt, options)
            .unwrap();
    lqr_riccati.solve(&state_0).unwrap();
    let x_traj_riccati = lqr_riccati.rollout(&state_0);
    let u_traj_riccati = lqr_riccati.get_u_traj();
    dbg!(&x_traj_riccati);
}

#[test]
fn test_linearize2_quadrotor() {
    let m = 1.0;
    let l = 0.3;
    let j = 0.2 * m * l * l;

    let pos_x = 0.0;
    let pos_y = 0.0;
    let theta = 0.0;
    let v_x = 0.0;
    let v_y = 0.0;
    let omega = 0.0;

    let registry = Arc::new(ExprRegistry::new());
    let state_hover = LtiState::<6, 0>::new([pos_x, pos_y, theta, v_x, v_y, omega]);
    let input_hover = LtiInput::<2, 0>::new([0.5 * m * c::GRAVITY, 0.5 * m * c::GRAVITY]);

    let model = Quadrotor2D::new(m, j, l, Some(&registry));
    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();

    let df_dx = integrator.jacobian_x().unwrap();
    let df_du = integrator.jacobian_u().unwrap();

    let dt = 0.05;
    let mut vals = state_hover.to_vec();
    vals.extend(input_hover.to_vec());
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);

    let a_mat: DMatrix<f64> = df_dx.evaluate(&vals).unwrap();
    let b_mat = df_du.evaluate(&vals).unwrap();

    dbg!(&a_mat, &b_mat);
}
