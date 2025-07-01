use nalgebra::DVector;

/// Computes the Kronecker product of two vectors.
///
/// The Kronecker product is a mathematical operation that takes two vectors
/// and produces a new vector by multiplying each element of the first vector
/// by the entire second vector.
///
/// # Arguments
///
/// * `a` - A reference to the first vector (`DVector<f64>`).
/// * `b` - A reference to the second vector (`DVector<f64>`).
///
/// # Returns
///
/// A new `DVector<f64>` containing the Kronecker product of `a` and `b`.
///
/// # Example
///
/// ```rust
/// use nalgebra::DVector;
/// use general::vector::kron;
///
/// let a = DVector::from_vec(vec![1.0, 2.0]);
/// let b = DVector::from_vec(vec![3.0, 4.0]);
/// let result = kron(&a, &b);
///
/// assert_eq!(result, DVector::from_vec(vec![3.0, 4.0, 6.0, 8.0]));
/// ```
pub fn kron(a: &DVector<f64>, b: &DVector<f64>) -> DVector<f64> {
    let mut result = DVector::<f64>::zeros(a.len() * b.len());

    for (i, &ai) in a.iter().enumerate() {
        let start = i * b.len();
        result.rows_mut(start, b.len()).copy_from(&(b * ai));
    }

    result
}

/// Vertically stacks two vectors, where the second vector is optional.
///
/// This function concatenates the first vector with the second vector (if it exists)
/// along the vertical axis. If the second vector is `None`, the function simply
/// returns the first vector.
///
/// # Arguments
///
/// * `a` - The first vector (`DVector<f64>`).
/// * `b` - An optional second vector (`Option<DVector<f64>`).
///
/// # Returns
///
/// A new `DVector<f64>` containing the vertically stacked result.
///
/// # Example
///
/// ```rust
/// use nalgebra::DVector;
/// use general::vector::vstack_option;
///
/// let a = DVector::from_vec(vec![1.0, 2.0]);
/// let b = Some(DVector::from_vec(vec![3.0, 4.0]));
/// let result = vstack_option(a.clone(), b);
///
/// assert_eq!(result, DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]));
///
/// let result_none = vstack_option(a.clone(), None);
/// assert_eq!(result_none, a);
/// ```
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kron_basic() {
        let a = DVector::from_vec(vec![1.0, 2.0]);
        let b = DVector::from_vec(vec![3.0, 4.0]);
        let result = kron(&a, &b);
        assert_eq!(result, DVector::from_vec(vec![3.0, 4.0, 6.0, 8.0]));
    }

    #[test]
    fn test_kron_empty_vectors() {
        let a = DVector::from_vec(vec![]);
        let b = DVector::from_vec(vec![]);
        let result = kron(&a, &b);
        assert_eq!(result, DVector::from_vec(vec![]));
    }

    #[test]
    fn test_kron_single_element_vectors() {
        let a = DVector::from_vec(vec![2.0]);
        let b = DVector::from_vec(vec![3.0]);
        let result = kron(&a, &b);
        assert_eq!(result, DVector::from_vec(vec![6.0]));
    }

    #[test]
    fn test_vstack_option_with_some() {
        let a = DVector::from_vec(vec![1.0, 2.0]);
        let b = Some(DVector::from_vec(vec![3.0, 4.0]));
        let result = vstack_option(a.clone(), b);
        assert_eq!(result, DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]));
    }

    #[test]
    fn test_vstack_option_with_none() {
        let a = DVector::from_vec(vec![1.0, 2.0]);
        let result = vstack_option(a.clone(), None);
        assert_eq!(result, a);
    }

    #[test]
    fn test_vstack_option_empty_vectors() {
        let a = DVector::from_vec(vec![]);
        let b = Some(DVector::from_vec(vec![]));
        let result = vstack_option(a.clone(), b);
        assert_eq!(result, DVector::from_vec(vec![]));
    }

    #[test]
    fn test_vstack_option_with_empty_second_vector() {
        let a = DVector::from_vec(vec![1.0, 2.0]);
        let b = Some(DVector::from_vec(vec![]));
        let result = vstack_option(a.clone(), b);
        assert_eq!(result, a);
    }
}
