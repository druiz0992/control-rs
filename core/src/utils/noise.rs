use nalgebra::DVector;
use rand_distr::{Distribution, Normal};

/// Represents a source of noise with a specified mean and standard deviation.
///
/// # Fields
/// - `std_dev`: The standard deviation of the noise distribution.
/// - `distribution`: The underlying normal distribution used to generate noise.
pub struct NoiseSource {
    std_dev: f64,

    distribution: Normal<f64>,
}

impl NoiseSource {
    /// Creates a new `NoiseSource` with the specified mean and standard deviation.
    ///
    /// - Parameters:
    ///   - `std_dev`: The standard deviation of the noise.
    /// - Returns:
    ///   - `Ok(Self)`: If the noise source is successfully created.
    ///   - `Err(ModelError)`: If the standard deviation is negative or the distribution fails to initialize.
    pub fn new(std_dev: f64) -> Result<Self, String> {
        let distribution = Normal::new(0.0, std_dev).map_err(|e| e.to_string())?;
        if std_dev < 0.0 {
            return Err("Noise stdev cannot be negative".into());
        }

        Ok(Self {
            std_dev,
            distribution,
        })
    }

    /// Adds noise to a given sample vector.
    ///
    /// - Parameters:
    ///   - `sample`: A `DVector<f64>` representing the input sample.
    /// - Returns:
    ///   - A `DVector<f64>` with noise added. If the standard deviation and mean are both zero, the original sample is returned unchanged.
    pub fn add_noise(&self, sample: DVector<f64>) -> DVector<f64> {
        if self.std_dev == 0.0 {
            return sample;
        }
        let noise: DVector<f64> = DVector::from_fn(sample.len(), |_, _| {
            self.distribution.sample(&mut rand::thread_rng())
        });
        sample + noise
    }
    pub fn sample(&self) -> f64 {
        if self.std_dev == 0.0 {
            return 0.0;
        }
        self.distribution.sample(&mut rand::thread_rng())
    }
}

/// Represents a collection of multiple noise sources.
pub struct NoiseSources(Vec<NoiseSource>);

impl NoiseSources {
    /// Creates a `NoiseSources` instance from a vector of noise statistics.
    ///
    /// - Parameters:
    ///   - `noise_statistics`: A vector of tuples where each tuple contains the mean and standard deviation for a noise source.
    /// - Returns:
    ///   - `Ok(NoiseSources)`: If all noise sources are successfully created.
    ///   - `Err(ModelError)`: If any noise source fails to initialize.
    pub fn from_stats(noise_statistics: Vec<f64>) -> Result<NoiseSources, String> {
        let sources: Vec<NoiseSource> = noise_statistics
            .into_iter()
            .map(NoiseSource::new)
            .collect::<Result<Vec<_>, String>>()?;
        Ok(NoiseSources(sources))
    }

    /// Adds noise to a sample using a specific noise source by index.
    ///
    /// - Parameters:
    ///   - `source_idx`: The index of the noise source to use.
    ///   - `sample`: A `DVector<f64>` representing the input sample.
    /// - Returns:
    ///   - `Ok(DVector<f64>)`: The sample with noise added.
    ///   - `Err(ModelError)`: If the specified noise source index is out of bounds.
    pub fn add_noise(&self, sample: DVector<f64>) -> Result<DVector<f64>, String> {
        if sample.len() != self.0.len() {
            return Err("Noise source size mismatch.".into());
        }
        let mut noise_sample = Vec::new();
        for noise_source in &self.0 {
            noise_sample.push(noise_source.sample());
        }

        Ok(sample + DVector::from_column_slice(&noise_sample))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_noise_source_creation() {
        let noise_source = NoiseSource::new(1.0);
        assert!(noise_source.is_ok());
    }

    #[test]
    fn test_noise_source_creation_negative_std_dev() {
        let noise_source = NoiseSource::new(-1.0);
        assert!(noise_source.is_err());
    }

    #[test]
    fn test_add_noise_with_zero_std_dev() {
        let noise_source = NoiseSource::new(0.0).unwrap();
        let sample = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let noisy_sample = noise_source.add_noise(sample.clone());
        assert_eq!(sample, noisy_sample);
    }

    #[test]
    fn test_add_noise_with_non_zero_std_dev() {
        let noise_source = NoiseSource::new(1.0).unwrap();
        let sample = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let noisy_sample = noise_source.add_noise(sample.clone());
        assert_ne!(sample, noisy_sample); // Noise should alter the sample
    }

    #[test]
    fn test_noise_sources_creation_from_stats() {
        let stats = vec![1.0, 2.0];
        let noise_sources = NoiseSources::from_stats(stats);
        assert!(noise_sources.is_ok());
    }

    #[test]
    fn test_noise_sources_creation_with_invalid_stats() {
        let stats = vec![-1.0, 2.0];
        let noise_sources = NoiseSources::from_stats(stats);
        assert!(noise_sources.is_err());
    }

    #[test]
    fn test_add_noise_with_valid_source() {
        let stats = vec![1.0, 2.0, 2.0];
        let noise_sources = NoiseSources::from_stats(stats).unwrap();
        let sample = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let noisy_sample = noise_sources.add_noise(sample.clone());
        assert!(noisy_sample.is_ok());
        assert_ne!(sample, noisy_sample.unwrap());
    }

    #[test]
    fn test_add_noise_with_invalid_source_number() {
        let stats = vec![1.0, 2.0];
        let noise_sources = NoiseSources::from_stats(stats).unwrap();
        let sample = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let noisy_sample = noise_sources.add_noise(sample.clone());
        assert!(noisy_sample.is_err());
    }
}
