use crate::{
    Minimizer, OSQPBuilder, SolverError, dtos::LagrangianMultiplier, osqp::solver::OSQPSolverHandle,
};
use general::matrix::dmat_to_vec;
use nalgebra::{DMatrix, DVector};
use osqp::{CscMatrix, Settings};

pub struct OSQPCluster {
    solvers: Vec<OSQPSolverHandle>, // OSQP solver instance per timestep
    primal_warm_starts: Vec<Option<Vec<f64>>>, // primal warm start vectors
    dual_warm_starts: Vec<Option<Vec<f64>>>, // dual warm start vectors (for constraints)
    options: Settings,
}

impl OSQPCluster {
    pub fn new(n_steps: usize, settings: Option<Settings>) -> Self {
        Self {
            solvers: Vec::with_capacity(n_steps),
            primal_warm_starts: vec![None; n_steps],
            dual_warm_starts: vec![None; n_steps],
            options: settings.unwrap_or_default(),
        }
    }

    pub fn initialize(&mut self, step_idx: usize, nu: usize) -> Result<(), SolverError> {
        use osqp::CscMatrix;

        let identity = DMatrix::<f64>::identity(nu, nu);
        let ones = DMatrix::<f64>::from_element(nu, nu, 1.0);

        // Prepare OSQP data matrices
        let q = CscMatrix::from_column_iter(nu, nu, ones.iter().cloned()).into_upper_tri();
        let q_vec = DVector::<f64>::zeros(nu); // will be updated in solve

        let lb_vec = DVector::<f64>::from_element(nu, f64::NEG_INFINITY);
        let ub_vec = DVector::<f64>::from_element(nu, f64::INFINITY);

        let builder = OSQPBuilder::new()
            .q_csc(q)
            .q_vec(q_vec)
            .a_mat(identity)
            .bounds_vec(lb_vec, ub_vec)
            .add_options(self.options.clone());

        let (handler, _) = builder.build()?;

        // Insert solver in vector
        if step_idx < self.solvers.len() {
            self.solvers[step_idx] = handler
        } else {
            self.solvers.push(handler);
        }
        Ok(())
    }

    pub fn solve_step(
        &mut self,
        step_idx: usize,
        q: &DMatrix<f64>,
        q_vec: &DVector<f64>,
        x_min: &DVector<f64>,
        x_max: &DVector<f64>,
    ) -> Result<DVector<f64>, SolverError> {
        if step_idx >= self.solvers.len() {
            return Err(SolverError::Other("Out of bounds solver".into()));
        }
        let solver = &mut self.solvers[step_idx];

        // Update linear cost q in OSQP
        // OSQP expects q as a slice, so convert &DVector<f64> accordingly
        solver.update_lin_cost(q_vec.as_slice());

        // Update bounds l, u if necessary (if u_nominal changed)
        let l = x_min;
        let u = x_max;
        solver.update_bounds(l.as_slice(), u.as_slice());

        // Warm start primal and dual variables if available
        if let Some(ref primal) = self.primal_warm_starts[step_idx] {
            solver.warm_start_primal(primal);
        }
        if let Some(ref dual) = self.dual_warm_starts[step_idx] {
            solver.warm_start_dual(dual);
        }

        let p = CscMatrix::from(&dmat_to_vec(q)).into_upper_tri();
        solver.update_p(p);

        // Solve
        let (primal, _, dual_mu, dual_lambdas) = solver.minimize(&[])?;

        // Store primal and dual for warm start next iteration
        self.primal_warm_starts[step_idx] = Some(primal.clone());
        self.dual_warm_starts[step_idx] = match (dual_mu, dual_lambdas) {
            (LagrangianMultiplier::Mus(mu), LagrangianMultiplier::Lambdas(lambda)) => {
                let mut combined = mu;
                combined.extend(lambda);
                Some(combined)
            }
            (LagrangianMultiplier::Mus(mu), _) => Some(mu),
            (_, LagrangianMultiplier::Lambdas(lambda)) => Some(lambda),
            _ => None,
        };

        Ok(DVector::from_column_slice(&primal))
    }
}
