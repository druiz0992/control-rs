
/// Represents the energy of a physical system, including both kinetic and potential components.
///
/// # Fields
/// - `kinetic`: The kinetic energy of the system, represented as a `f64`.
/// - `potential`: The potential energy of the system, represented as a `f64`.
///
/// # Methods
/// 
/// ## Constructors
/// - `new(kinetic: f64, potential: f64) -> Self`
///   - Creates a new `Energy` instance with the specified kinetic and potential energy values.
///
/// ## Accessors
/// - `total(&self) -> f64`
///   - Returns the total energy of the system, which is the sum of kinetic and potential energy.
/// - `get_kinetic(&self) -> f64`
///   - Returns the kinetic energy of the system.
/// - `get_potential(&self) -> f64`
///   - Returns the potential energy of the system.
///
/// ## Mutators
/// - `set_kinetic(&mut self, kinetic: f64) -> Result<(), &'static str>`
///   - Sets the kinetic energy of the system. Returns an error if the provided value is negative.
/// - `set_potential(&mut self, potential: f64) -> Result<(), &'static str>`
///   - Sets the potential energy of the system. Returns an error if the provided value is negative.
///
/// # Panics
/// None of the methods in this struct cause panics. However, the `set_kinetic` and `set_potential` methods
/// return an error if the provided energy values are negative.
///
/// # Examples
/// ```
/// use control_rs::physics::energy::Energy;
/// 
/// let mut energy = Energy::new(10.0, 20.0);
/// assert_eq!(energy.total(), 30.0);
///
/// energy.set_kinetic(15.0).unwrap();
/// assert_eq!(energy.get_kinetic(), 15.0);
///
/// energy.set_potential(25.0).unwrap();
/// assert_eq!(energy.get_potential(), 25.0);
/// ```
#[derive(Clone, Debug)]
pub struct Energy {
    kinetic: f64,
    potential: f64,
}

impl Energy {
    pub fn new(kinetic: f64, potential: f64) -> Self {
        Energy { kinetic, potential }
    }

    pub fn total(&self) -> f64 {
        self.kinetic + self.get_potential()
    }

    pub fn get_kinetic(&self) -> f64 {
        self.kinetic
    }

    pub fn get_potential(&self) -> f64 {
        self.potential
    }

    pub fn set_kinetic(&mut self, kinetic: f64) -> Result<(), &'static str> {
        if kinetic < 0.0 {
            return Err("Kinetic energy cannot be negative");
        }
        Ok(self.kinetic = kinetic)
    }

    pub fn set_potential(&mut self, potential: f64) -> Result<(), &'static str> {
        if potential < 0.0 {
            return Err("Potential energy cannot be negative");
        }
        Ok(self.potential = potential)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_energy_creation() {
        let energy = Energy::new(10.0, 20.0);
        assert_eq!(energy.get_kinetic(), 10.0);
        assert_eq!(energy.get_potential(), 20.0);
        assert_eq!(energy.total(), 30.0);
    }

    #[test]
    fn test_set_kinetic() {
        let mut energy = Energy::new(10.0, 20.0);
        assert!(energy.set_kinetic(15.0).is_ok());
        assert_eq!(energy.get_kinetic(), 15.0);
    }

    #[test]
    fn test_set_potential() {
        let mut energy = Energy::new(10.0, 20.0);
        assert!(energy.set_potential(25.0).is_ok());
        assert_eq!(energy.get_potential(), 25.0);
    }

    #[test]
    fn test_negative_kinetic_error() {
        let mut energy = Energy::new(10.0, 20.0);
        assert!(energy.set_kinetic(-5.0).is_err());
        assert_eq!(energy.get_kinetic(), 10.0);
    }

    #[test]
    fn test_negative_potential_error() {
        let mut energy = Energy::new(10.0, 20.0);
        assert!(energy.set_potential(-10.0).is_err());
        assert_eq!(energy.get_potential(), 20.0);
    }

    #[test]
    fn test_total_energy() {
        let energy = Energy::new(5.0, 15.0);
        assert_eq!(energy.total(), 20.0);
    }

    #[test]
    fn test_zero_energy() {
        let energy = Energy::new(0.0, 0.0);
        assert_eq!(energy.get_kinetic(), 0.0);
        assert_eq!(energy.get_potential(), 0.0);
        assert_eq!(energy.total(), 0.0);
    }

    #[test]
    fn test_large_energy_values() {
        let energy = Energy::new(1e10, 2e10);
        assert_eq!(energy.get_kinetic(), 1e10);
        assert_eq!(energy.get_potential(), 2e10);
        assert_eq!(energy.total(), 3e10);
    }
}
