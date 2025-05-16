
pub fn unzip3<A, B, C>(v: Vec<(A, B, C)>) -> (Vec<A>, Vec<B>, Vec<C>) {
    let mut a = Vec::with_capacity(v.len());
    let mut b = Vec::with_capacity(v.len());
    let mut c = Vec::with_capacity(v.len());

    for (x, y, z) in v {
        a.push(x);
        b.push(y);
        c.push(z);
    }

    (a, b, c)
}


pub fn within_tolerance(param1: f64, param2: f64, tol: f64) -> bool {
    let diff = (param1 - param2).abs();
    let scale = param1.abs().max(param2.abs());

    diff <= tol * scale.max(1.0)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unzip3() {
        let input = vec![(1, 'a', 2.0), (2, 'b', 3.0), (3, 'c', 4.0)];
        let (a, b, c) = unzip3(input);

        assert_eq!(a, vec![1, 2, 3]);
        assert_eq!(b, vec!['a', 'b', 'c']);
        assert_eq!(c, vec![2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_within_tolerance_exact_match() {
        assert!(within_tolerance(10.0, 10.0, 0.1));
    }

    #[test]
    fn test_within_tolerance_within_range() {
        assert!(within_tolerance(10.0, 10.5, 0.1));
    }

    #[test]
    fn test_within_tolerance_out_of_range() {
        assert!(!within_tolerance(10.0, 11.0, 1e-5));
    }

    #[test]
    fn test_within_tolerance_with_zero_tolerance() {
        assert!(!within_tolerance(10.0, 10.1, 0.0));
    }

    #[test]
    fn test_within_tolerance_negative_numbers() {
        assert!(within_tolerance(-10.0, -10.5, 0.1));
        assert!(!within_tolerance(-10.0, -11.0, 0.01));
    }
}
