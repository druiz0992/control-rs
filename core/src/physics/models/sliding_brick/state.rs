use crate::physics::traits::State;
use macros::StateOps;

#[derive(Clone, Debug, StateOps)]
pub struct SlidingBrickState {
    pub pos_x: f64,
    pub pos_y: f64,
    #[constrained]
    pub v_x: f64,
    #[constrained]
    pub v_y: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let state = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.pos_x, 1.0);
        assert_eq!(state.pos_y, 2.0);
        assert_eq!(state.v_x, 3.0);
        assert_eq!(state.v_y, 4.0);
    }

    #[test]
    fn test_state() {
        let state = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.state(), (1.0, 2.0, 3.0, 4.0));
    }

    #[test]
    fn test_as_vec() {
        let state = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.as_vec(), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_from_vec() {
        let vec = vec![1.0, 2.0, 3.0, 4.0];
        let state = SlidingBrickState::from_vec(vec);
        assert_eq!(state.pos_x, 1.0);
        assert_eq!(state.pos_y, 2.0);
        assert_eq!(state.v_x, 3.0);
        assert_eq!(state.v_y, 4.0);
    }

    #[test]
    fn test_labels() {
        let labels = SlidingBrickState::labels();
        assert_eq!(labels, &["pos_x", "pos_y", "v_x", "v_y"]);
    }

    #[test]
    fn test_add() {
        let state1 = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = SlidingBrickState::new(0.5, 1.5, 2.5, 3.5);
        let result = state1 + state2;
        assert_eq!(result.pos_x, 1.5);
        assert_eq!(result.pos_y, 3.5);
        assert_eq!(result.v_x, 5.5);
        assert_eq!(result.v_y, 7.5);
    }

    #[test]
    fn test_sub() {
        let state1 = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = SlidingBrickState::new(0.5, 1.5, 2.5, 3.5);
        let result = state1 - state2;
        assert_eq!(result.pos_x, 0.5);
        assert_eq!(result.pos_y, 0.5);
        assert_eq!(result.v_x, 0.5);
        assert_eq!(result.v_y, 0.5);
    }

    #[test]
    fn test_mul() {
        let state = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let result = state * 2.0;
        assert_eq!(result.pos_x, 2.0);
        assert_eq!(result.pos_y, 4.0);
        assert_eq!(result.v_x, 6.0);
        assert_eq!(result.v_y, 8.0);
    }

    #[test]
    fn test_div() {
        let state = SlidingBrickState::new(2.0, 4.0, 6.0, 8.0);
        let result = state / 2.0;
        assert_eq!(result.pos_x, 1.0);
        assert_eq!(result.pos_y, 2.0);
        assert_eq!(result.v_x, 3.0);
        assert_eq!(result.v_y, 4.0);
    }

    #[test]
    fn test_eq() {
        let state1 = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let state2 = SlidingBrickState::new(0.5, 1.5, 2.5, 3.5);
        assert!(state1 != state2);
    }

    #[test]
    fn test_get_q() {
        let state1 = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let q = state1.get_q();

        assert!(q[0] == 1.0);
        assert!(q[1] == 2.0);
        assert!(q.len() == 2);
    }

    #[test]
    fn test_get_v() {
        let state1 = SlidingBrickState::new(1.0, 2.0, 3.0, 4.0);
        let v = state1.get_v();

        assert!(v[0] == 3.0);
        assert!(v[1] == 4.0);
        assert!(v.len() == 2);
    }

    #[test]
    fn test_dim_q() {
        assert!(SlidingBrickState::dim_q() == 2);
    }

    #[test]
    fn test_dim_v() {
        assert!(SlidingBrickState::dim_v() == 2);
    }
}
