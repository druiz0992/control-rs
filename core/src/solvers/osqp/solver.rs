use crate::physics::ModelError;
use crate::solvers::Minimizer;
use crate::solvers::dtos::{
    KktConditionsStatus,
    {LagrangianMultiplier, SolverResult},
};
use osqp::{CscMatrix, Problem, Status};
use std::sync::{Arc, Mutex};

pub struct OSQPSolver {
    pub(super) problem: Problem,
    pub(super) n_eq: usize,
    pub(super) n_ineq: usize,
}

impl OSQPSolver {
    pub fn get_n_eq(&self) -> usize {
        self.n_eq
    }

    pub fn get_n_ineq(&self) -> usize {
        self.n_ineq
    }
}
pub struct OSQPSolverHandle {
    solver: Arc<Mutex<OSQPSolver>>,
}

impl OSQPSolverHandle {
    pub fn new(solver: OSQPSolver) -> Self {
        Self {
            solver: Arc::new(Mutex::new(solver)),
        }
    }

    pub fn update_lin_cost(&self, q: &[f64]) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_lin_cost(q);
    }
    pub fn update_p(&self, p: CscMatrix) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_P(p)
    }
    pub fn update_a(&self, a: CscMatrix) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_A(a)
    }
    pub fn update_lower_bound(&self, l: &[f64]) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_lower_bound(l);
    }
    pub fn update_upper_bound(&self, u: &[f64]) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_upper_bound(u);
    }
    pub fn update_bounds(&self, l: &[f64], u: &[f64]) {
        let mut h = self.solver.lock().unwrap();
        h.problem.update_bounds(l, u);
    }
}

impl OSQPSolver {
    pub fn solve(&mut self) -> Status {
        self.problem.solve()
    }
}

impl Minimizer for OSQPSolverHandle {
    fn minimize(&self, _initial_guess: &[f64]) -> Result<SolverResult, ModelError> {
        let mut locked_solver = self.solver.lock().unwrap();
        let n_eq = locked_solver.n_eq;
        let status = locked_solver.solve();
        let (sol, kkt_conditions, lm_mus, lm_lambdas) = status_to_result(status, n_eq)?;

        Ok((sol, kkt_conditions, lm_mus, lm_lambdas))
    }
}

fn status_to_result(status: Status, n_eq: usize) -> Result<SolverResult, ModelError> {
    match status {
        Status::Solved(solution)
        | Status::SolvedInaccurate(solution)
        | Status::MaxIterationsReached(solution)
        | Status::TimeLimitReached(solution) => {
            let primal = solution.x().to_vec();
            let lm = solution.y().to_vec();
            let lm_mus = LagrangianMultiplier::Mus(lm[..n_eq].to_vec());
            let lm_lambdas = LagrangianMultiplier::Mus(lm[n_eq..].to_vec());

            let kkt_conditions = KktConditionsStatus {
                stationarity: 0.0,
                max_primal_feasibility_c: None,
                min_primal_feasibility_h: Some(solution.pri_res()),
                dual_feasibility: Some(solution.dua_res()),
                complementary_slackness: None,
            };

            Ok((primal, kkt_conditions, lm_mus, lm_lambdas))
        }

        Status::PrimalInfeasible(_)
        | Status::PrimalInfeasibleInaccurate(_)
        | Status::DualInfeasible(_)
        | Status::DualInfeasibleInaccurate(_)
        | Status::NonConvex(_) => Err(ModelError::SolverError(
            "OSQP: Problem is infeasible or non-convex".into(),
        )),

        Status::__Nonexhaustive => Err(ModelError::SolverError(
            "OSQP: Unknown solver status".into(),
        )),
    }
}
#[cfg(test)]
mod tests {
    use osqp::{CscMatrix, Settings};

    use super::*;

    #[test]
    fn test_osqp() {
        let p_mat = &[[4.0, 1.0], [1.0, 2.0]];
        let q = &[1.0, 1.0];
        let a_mat = &[[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        let l = &[1.0, 0.0, 0.0];
        let u = &[1.0, 0.7, 0.7];

        let p_mat = CscMatrix::from(p_mat).into_upper_tri();

        let settings = Settings::default().verbose(false);

        let mut prob =
            Problem::new(p_mat, q, a_mat, l, u, &settings).expect("failed to setup problem");

        let result = prob.solve();

        dbg!(result.x().expect("failed to solve problem"));
    }

    #[test]
    fn test_osqp_no_equality() {
        let p_mat = &[[4.0, 1.0], [1.0, 2.0]];
        let q = &[1.0, 1.0];
        let a_mat = &[[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        let l = &[0.0, 0.0, 0.0];
        let u = &[1.0, 0.7, 0.7];

        let p_mat = CscMatrix::from(p_mat).into_upper_tri();

        let settings = Settings::default().verbose(false);

        let mut prob =
            Problem::new(p_mat, q, a_mat, l, u, &settings).expect("failed to setup problem");

        let result = prob.solve();

        dbg!(result.x().expect("failed to solve problem"));
    }

    #[test]
    fn test_osqp_no_inequality() {
        let p_mat = &[
            [4.0, 1.0, 3.0, 2.0],
            [1.0, 2.0, 3.0, 1.0],
            [3.0, 3.0, 4.0, 2.0],
            [2.0, 1.0, 2.0, 6.0],
        ];
        let q = &[1.0, 1.0, 2.0, 3.0];
        let a_mat = &[[1.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 1.0]];
        let l = &[1.0, 0.7];
        let u = &[1.0, 0.7];

        let p_mat = CscMatrix::from(p_mat).into_upper_tri();

        let settings = Settings::default().verbose(false);

        let mut prob =
            Problem::new(p_mat, q, a_mat, l, u, &settings).expect("failed to setup problem");

        let result = prob.solve();

        dbg!(result.x().expect("failed to solve problem"));
    }

    #[test]
    fn test_osqp_handle() {
        // Define problem data
        let p_mat = &[[4.0, 1.0], [1.0, 2.0]];
        let q = &[1.0, 1.0];
        let a_mat = &[[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        let l = &[1.0, 0.0, 0.0];
        let u = &[1.0, 0.7, 0.7];

        // Extract the upper triangular elements of `P`
        let p_mat = CscMatrix::from(p_mat).into_upper_tri();

        // Disable verbose output
        let settings = Settings::default().verbose(false);

        // Create an OSQP problem
        let prob = Problem::new(p_mat, q, a_mat, l, u, &settings).expect("failed to setup problem");

        let handle = OSQPSolverHandle::new(OSQPSolver {
            problem: prob,
            n_eq: 0,
            n_ineq: l.len() + u.len(),
        });
        let result = handle.minimize(&[]).unwrap();

        // Print the solution
        dbg!(result.0);
    }

    #[test]
    fn test_osqp2() {
        // Quadratic matrix P (must be upper triangular for OSQP)
        let p = vec![
            vec![1.0, 0.3, 0.0, 0.0],
            vec![0.3, 1.0, 0.0, 0.0],
            vec![0.0, 0.0, 2.0, 0.0],
            vec![0.0, 0.0, 0.0, 4.0],
        ];
        let p = CscMatrix::from(&p).into_upper_tri();

        // Linear term q
        let q = vec![-2.0, 3.4, 2.0, 4.0];

        // Constraint matrix A
        let a = vec![
            vec![0.0, 0.0, 1.0, 1.0],   // a + b = 1
            vec![-1.0, 2.3, 1.0, -2.0], // -y1 + 2.3y2 + a - 2b = 3
            vec![1.0, 0.0, 0.0, 0.0],   // y1 lower
            vec![0.0, 1.0, 0.0, 0.0],   // y2 lower
            vec![0.0, 0.0, 1.0, 0.0],   // a lower
            vec![0.0, 0.0, 0.0, 1.0],   // b lower
        ];
        let a = CscMatrix::from(&a);

        // Constraint bounds
        let l = vec![1.0, 3.0, -0.5, -0.5, -1.0, -1.0];
        let u = vec![1.0, 3.0, 1.0, 1.0, 1.0, 1.0];

        // Settings (can tweak tolerances here)
        let settings = Settings::default().verbose(false);

        // Build problem
        let mut prob = Problem::new(p, &q, a, &l, &u, &settings).unwrap();

        // Solve
        let result = prob.solve();

        println!("Status: {:?}", result);
    }
}
