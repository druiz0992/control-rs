use nalgebra::DVector;

/// Kronecker product
pub fn kron(a: &DVector<f64>, b: &DVector<f64>) -> DVector<f64> {
    let mut result = DVector::<f64>::zeros(a.len() * b.len());

    for (i, &ai) in a.iter().enumerate() {
        let start = i * b.len();
        result.rows_mut(start, b.len()).copy_from(&(b * ai));
    }

    result
}

/// Vertically stacks two `Option<DVector<f64>>`.
pub fn vstack_option(a: DVector<f64>, b: Option<DVector<f64>>) -> DVector<f64> {
    match b {
        Some(vec_b) => {
            let mut stacked = DVector::zeros(a.len() + vec_b.len());
            stacked.rows_mut(0, a.len()).copy_from(&a);
            stacked.rows_mut(a.len(), vec_b.len()).copy_from(&vec_b);
            stacked
        }
        None => a,
    }
}
