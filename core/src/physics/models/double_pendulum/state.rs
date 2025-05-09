use crate::physics::traits::State;
use macros::StateOps;

#[derive(Clone, Debug, StateOps)]
pub struct DoublePendulumState {
    pub theta1: f64,
    pub omega1: f64,
    pub theta2: f64,
    pub omega2: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let state = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.theta1, 1.0);
        assert_eq!(state.omega1, 2.0);
        assert_eq!(state.theta2, 3.0);
        assert_eq!(state.omega2, 4.0);
    }

    #[test]
    fn test_state() {
        let state = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.state(), (1.0, 2.0, 3.0, 4.0));
    }

    #[test]
    fn test_as_vec() {
        let state = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.as_vec(), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_from_vec() {
        let vec = vec![1.0, 2.0, 3.0, 4.0];
        let state = DoublePendulumState::from_vec(vec);
        assert_eq!(state.theta1, 1.0);
        assert_eq!(state.omega1, 2.0);
        assert_eq!(state.theta2, 3.0);
        assert_eq!(state.omega2, 4.0);
    }

    #[test]
    fn test_labels() {
        let labels = DoublePendulumState::labels();
        assert_eq!(labels, &["theta1", "omega1", "theta2", "omega2"]);
    }

    #[test]
    fn test_add() {
        let state1 = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = DoublePendulumState::new(0.5, 1.5, 2.5, 3.5);
        let result = state1 + state2;
        assert_eq!(result.theta1, 1.5);
        assert_eq!(result.omega1, 3.5);
        assert_eq!(result.theta2, 5.5);
        assert_eq!(result.omega2, 7.5);
    }

    #[test]
    fn test_sub() {
        let state1 = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = DoublePendulumState::new(0.5, 1.5, 2.5, 3.5);
        let result = state1 - state2;
        assert_eq!(result.theta1, 0.5);
        assert_eq!(result.omega1, 0.5);
        assert_eq!(result.theta2, 0.5);
        assert_eq!(result.omega2, 0.5);
    }

    #[test]
    fn test_mul() {
        let state = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let result = state * 2.0;
        assert_eq!(result.theta1, 2.0);
        assert_eq!(result.omega1, 4.0);
        assert_eq!(result.theta2, 6.0);
        assert_eq!(result.omega2, 8.0);
    }

    #[test]
    fn test_div() {
        let state = DoublePendulumState::new(2.0, 4.0, 6.0, 8.0);
        let result = state / 2.0;
        assert_eq!(result.theta1, 1.0);
        assert_eq!(result.omega1, 2.0);
        assert_eq!(result.theta2, 3.0);
        assert_eq!(result.omega2, 4.0);
    }

    #[test]
    fn test_eq() {
        let state1 = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = DoublePendulumState::new(0.5, 1.5, 2.5, 3.5);
        assert!(state1 != state2);
    }

    #[test]
    fn test_get_q() {
        let state1 = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let q = state1.get_q();

        assert!(q[0] == 1.0);
        assert!(q[1] == 2.0);
        assert!(q[2] == 3.0);
        assert!(q[3] == 4.0);
        assert!(q.len() == 4);
    }

    #[test]
    fn test_get_v() {
        let state1 = DoublePendulumState::new(1.0, 2.0, 3.0, 4.0);
        let v = state1.get_v();

        assert!(v.len() == 0);
    }

    #[test]
    fn test_dim_q() {
        assert!(DoublePendulumState::dim_q() == 4);
    }

    #[test]
    fn test_dim_v() {
        assert!(DoublePendulumState::dim_v() == 0);
    }
}
