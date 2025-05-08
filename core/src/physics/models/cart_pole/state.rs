use crate::physics::traits::State;
use macros::StateOps;

#[derive(Clone, Debug, StateOps)]
pub struct CartPoleState {
    pub pos_x: f64,
    pub v_x: f64,
    pub theta: f64,
    pub omega: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cart_pole_state_initialization() {
        let state = CartPoleState {
            pos_x: 0.0,
            v_x: 0.0,
            theta: 0.0,
            omega: 0.0,
        };

        assert_eq!(state.pos_x, 0.0);
        assert_eq!(state.v_x, 0.0);
        assert_eq!(state.theta, 0.0);
        assert_eq!(state.omega, 0.0);
    }

    #[test]
    fn test_cart_pole_state_update() {
        let mut state = CartPoleState {
            pos_x: 1.0,
            v_x: 2.0,
            theta: 0.5,
            omega: 1.5,
        };

        state.pos_x += 0.1;
        state.v_x += 0.2;
        state.theta += 0.05;
        state.omega += 0.15;

        assert_eq!(state.pos_x, 1.1);
        assert_eq!(state.v_x, 2.2);
        assert_eq!(state.theta, 0.55);
        assert_eq!(state.omega, 1.65);
    }

    #[test]
    fn test_cart_pole_state_clone() {
        let state = CartPoleState {
            pos_x: 1.0,
            v_x: 2.0,
            theta: 0.5,
            omega: 1.5,
        };

        let cloned_state = state.clone();

        assert_eq!(cloned_state.pos_x, state.pos_x);
        assert_eq!(cloned_state.v_x, state.v_x);
        assert_eq!(cloned_state.theta, state.theta);
        assert_eq!(cloned_state.omega, state.omega);
    }

    #[test]
    fn test_cart_pole_state_debug_format() {
        let state = CartPoleState {
            pos_x: 1.0,
            v_x: 2.0,
            theta: 0.5,
            omega: 1.5,
        };

        let debug_string = format!("{:?}", state);
        assert!(debug_string.contains("CartPoleState"));
        assert!(debug_string.contains("pos_x: 1.0"));
        assert!(debug_string.contains("v_x: 2.0"));
        assert!(debug_string.contains("theta: 0.5"));
        assert!(debug_string.contains("omega: 1.5"));
    }
}
