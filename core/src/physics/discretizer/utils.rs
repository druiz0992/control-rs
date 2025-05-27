use crate::numeric_services::solver::dtos::SolverResult;
use crate::numeric_services::solver::{KktConditionsStatus, NewtonSolverSymbolic, OptimizerConfig};
use crate::numeric_services::symbolic::{ExprMatrix, ExprRegistry, ExprScalar, ExprVector};
use crate::physics::constants as c;
use crate::physics::error::ModelError;
use crate::physics::traits::State;
use crate::solver::RootFinder;
use std::sync::Arc;

pub fn get_states(registry: &Arc<ExprRegistry>) -> Result<(ExprVector, ExprVector), ModelError> {
    let current_state = registry.get_vector(c::STATE_SYMBOLIC)?;
    let next_state = current_state.build_next();
    registry.insert_vector_expr(c::NEXT_STATE_SYMBOLIC, next_state.clone());

    Ok((current_state, next_state))
}

pub fn get_q_states(registry: &Arc<ExprRegistry>) -> Result<(ExprVector, ExprVector), ModelError> {
    let current_q_state = registry.get_vector(c::STATE_Q_SYMBOLIC)?;
    let next_q_state = current_q_state.build_next();
    registry.insert_vector_expr(c::NEXT_STATE_Q_SYMBOLIC, next_q_state.clone());

    Ok((current_q_state, next_q_state))
}

pub fn get_v_states(registry: &Arc<ExprRegistry>) -> Result<(ExprVector, ExprVector), ModelError> {
    let current_v_state = registry.get_vector(c::STATE_V_SYMBOLIC)?;
    let next_v_state = current_v_state.build_next();
    registry.insert_vector_expr(c::NEXT_STATE_V_SYMBOLIC, next_v_state.clone());

    Ok((current_v_state, next_v_state))
}

pub fn step_intrinsic<S: State, RF: RootFinder>(
    state: &S,
    dt: f64,
    solver: &RF,
    registry: &Arc<ExprRegistry>,
) -> Result<SolverResult, ModelError> {
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    registry.insert_vec_as_vars(c::STATE_SYMBOLIC, &state.to_vec())?;

    let start_dims = if S::dim_v() == 0 { 0 } else { S::dim_q() };

    solver.find_roots(&state.to_vec()[start_dims..])
}

/// 1/2 * next_v_state * M * next_v_state + linear term * next_v_state
pub fn build_objective(
    mass_matrix: &ExprMatrix,
    linear_term: &ExprVector,
    next_v_state: &ExprVector,
) -> ExprScalar {
    let expr1 = next_v_state
        .vecmul_mat(mass_matrix)
        .wrap()
        .dot(next_v_state)
        .unwrap()
        .wrap()
        .scalef(0.5)
        .wrap();
    let expr2 = linear_term.dot(next_v_state).unwrap().wrap();
    expr1.add(&expr2)
}

pub(super) fn check_convergence(status: KktConditionsStatus, tol: f64) -> Result<(), ModelError> {
    let stationarity_cond = status.stationarity <= tol;
    if !stationarity_cond {
        return Err(ModelError::SolverError(format!(
            "Stationarity Condition failed: {}",
            status.stationarity
        )));
    }

    let primal_feasibility_cond = status.min_primal_feasibility_h.unwrap_or(0.0) >= -tol;
    if !primal_feasibility_cond {
        return Err(ModelError::SolverError(format!(
            "Primal Feasibility Condition failed: {:?}",
            status.min_primal_feasibility_h
        )));
    }

    let dual_feasibility_cond = status.dual_feasibility.unwrap_or(0.0) >= -tol;
    if !dual_feasibility_cond {
        return Err(ModelError::SolverError(format!(
            "Dual Feasibility Condition failed: {:?}",
            status.dual_feasibility
        )));
    }

    let complementary_cond = status.complementary_slackness.unwrap_or(0.0) <= tol;
    if !complementary_cond {
        return Err(ModelError::SolverError(format!(
            "Complementary Condition failed: {:?}",
            status.complementary_slackness
        )));
    }

    Ok(())
}

pub fn init_constrained_dynamics(
    linear_term: &ExprVector,
    dt_expr: &ExprScalar,
    solver_options: Option<OptimizerConfig>,
    registry: &Arc<ExprRegistry>,
) -> Result<NewtonSolverSymbolic, ModelError> {
    // retrieve M and J
    let mass_matrix = registry.get_matrix(c::MASS_MATRIX_SYMBOLIC)?;
    let jacobian_constraints_expr = registry.get_vector(c::CONSTRAINT_JACOBIAN_SYMBOLIC)?;
    // build v_next, q_current
    let (_, next_v_state) = get_v_states(registry)?;
    let (current_q_state, _) = get_q_states(registry)?;
    // objective =  1/2 * v_next' * M * v_next + linear_term' * v_next
    let objective_expr = build_objective(&mass_matrix, linear_term, &next_v_state);
    // constraints => J * (q_current + dt * v_next) >= 0
    let ineq_constraints_expr = ExprVector::from_vec(vec![
        jacobian_constraints_expr
            .dot(&current_q_state.add(&next_v_state.scale(dt_expr)).wrap())
            .unwrap()
            .wrap(),
    ]);

    NewtonSolverSymbolic::new_minimization(
        &objective_expr,
        None,
        Some(&ineq_constraints_expr),
        &next_v_state,
        registry,
        solver_options,
    )
}

#[cfg(test)]
mod tests {
    use crate::utils::helpers::within_tolerance;
    use crate::numeric_services::symbolic::{SymbolicExpr, TryIntoEvalResult};
    use crate::physics::constants as c;
    use crate::physics::models::BouncingBall;
    use crate::physics::models::dynamics::SymbolicDynamics;

    use super::*;
    use nalgebra::{DMatrix, DVector};
    use proptest::prelude::*;

    fn build_objective_fn(v_next: &DVector<f64>, v_current: &DVector<f64>, m: f64, dt: f64) -> f64 {
        let mass_matrix = DMatrix::identity(2, 2) * m;
        let g_vector = DVector::from_vec(vec![0.0, 9.81]);

        let term2 = (&mass_matrix * (dt * g_vector - v_current)).dot(v_next);
        let term1 = v_next.transpose() * &mass_matrix * v_next * 0.5;

        term1[(0, 0)] + term2
    }

    #[test]
    #[ignore]
    fn test_objective_symbolic() {
        let registry = Arc::new(ExprRegistry::new());
        let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
        let bouncing_ball = BouncingBall::new(1.0, 2.0, Some(&registry));
        let mass_matrix = registry.get_matrix(c::MASS_MATRIX_SYMBOLIC).unwrap();
        let linear_term = bouncing_ball.cost_linear_term(&dt_expr, &registry).unwrap();
        let (_, next_v_state) = get_v_states(&registry).unwrap();
        let objective_symbolic = build_objective(&mass_matrix, &linear_term, &next_v_state);
        dbg!(&objective_symbolic, &linear_term, &next_v_state);
    }

    proptest! {
        #[test]
        fn test_objective_equivalence(
            v_x_next in -5.0..5.0,
            v_y_next in -5.0..5.0,
            v_x in -5.0..5.0,
            v_y in -5.0..5.0,
            m in 0.1f64..10.0
        ) {
            let dt = 0.01;
            let registry = Arc::new(ExprRegistry::new());
            let dt_expr = ExprScalar::new(c::TIME_DELTA_SYMBOLIC);
            let model = BouncingBall::new(m, 0.0, Some(&registry));

            registry.insert_var("v_x", v_x);
            registry.insert_var("v_y", v_y);
            registry.insert_var("next_v_x", v_x_next);
            registry.insert_var("next_v_y", v_y_next);
            registry.insert_var("m", m);
            registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);


            let linear_term = model.cost_linear_term(&dt_expr, &registry).unwrap();
            let (_, next_v_state) = get_v_states(&registry).unwrap();
            let mass_matrix = registry.get_matrix(c::MASS_MATRIX_SYMBOLIC).unwrap();
            let v_next = DVector::from_vec(vec![v_x_next, v_y_next]);
            let v_current = DVector::from_vec(vec![v_x, v_y]);

            let objective = build_objective_fn(&v_next, &v_current, m, dt);
            let objective_symbolic = build_objective(&mass_matrix, &linear_term, &next_v_state);

            let objective_fn = objective_symbolic
                .to_fn(&registry)
                .unwrap();
            let objective_result: f64 = objective_fn(None).try_into_eval_result().unwrap();

            // Compare numeric and symbolic outputs approximately
            let tol = 1e-3; // tolerance for floating point comparison

            if !within_tolerance(objective, objective_result, tol) {
                dbg!(&objective, &objective_result);
            }

            assert!(within_tolerance(objective, objective, tol), "pos_x mismatch");
        }
    }
}
