use crate::utils::Labelizable;
use crate::physics::traits::State;
use macros::{ArrayLabelOps, ArrayStateOps};

#[derive(Clone, Debug, ArrayStateOps, ArrayLabelOps)]
pub struct LtiState<const N: usize, const C: usize> {
    pub data: [f64; N],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let state = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        assert_eq!(state.data, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_state() {
        let state = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let vals = state.extract(&["ltistate_0", "ltistate_1", "ltistate_2", "ltistate_3"]);
        assert_eq!(vals, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_to_vec() {
        let state = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        assert_eq!(state.to_vec(), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_from_vec() {
        let vec = vec![1.0, 2.0, 3.0, 4.0];
        let state = LtiState::<4, 0>::from_vec(vec);
        assert_eq!(state.data, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_labels() {
        let labels = LtiState::<4, 0>::labels();
        assert_eq!(
            labels,
            &["ltistate_0", "ltistate_1", "ltistate_2", "ltistate_3"]
        );
    }

    #[test]
    fn test_add() {
        let state1 = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let state2 = LtiState::<4, 0>::new([0.5, 1.5, 2.5, 3.5]);
        let result = state1 + state2;
        assert_eq!(result.data, [1.5, 3.5, 5.5, 7.5]);
    }

    #[test]
    fn test_sub() {
        let state1 = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let state2 = LtiState::<4, 0>::new([0.5, 1.5, 2.5, 3.5]);
        let result = state1 - state2;
        assert_eq!(result.data, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_mul() {
        let state = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let result = state * 2.0;
        assert_eq!(result.data, [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn test_div() {
        let state = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let result = state / 2.0;
        assert_eq!(result.data, [0.5, 1.0, 1.5, 2.0]);
    }

    #[test]
    fn test_ne() {
        let state1 = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let state2 = LtiState::<4, 0>::new([2.0, 3.0, 4.0, 5.0]);
        assert!(state1 != state2);
    }
    #[test]
    fn test_eq() {
        let state1 = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        let state2 = LtiState::<4, 0>::new([1.0, 2.0, 3.0, 4.0]);
        assert!(state1 == state2);
    }

    #[test]
    fn test_get_q() {
        let state = LtiState::<4, 2>::new([1.0, 2.0, 3.0, 4.0]);
        let q = state.get_q();

        assert!(q[0] == 1.0);
        assert!(q[1] == 2.0);
        assert!(q.len() == 2);
    }

    #[test]
    fn test_get_v() {
        let state = LtiState::<4, 2>::new([1.0, 2.0, 3.0, 4.0]);
        let v = state.get_v();

        assert!(v[0] == 3.0);
        assert!(v[1] == 4.0);
        assert!(v.len() == 2);
    }

    #[test]
    fn test_dim_q() {
        assert!(LtiState::<4, 2>::dim_q() == 2);
    }

    #[test]
    fn test_dim_v() {
        assert!(LtiState::<4, 2>::dim_v() == 2);
    }
}
