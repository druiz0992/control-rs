use nalgebra::DVector;
use rand_distr::{Distribution, Normal};

use crate::physics::ModelError;

pub struct NoiseSource {
    mean: f64,
    std_dev: f64,

    distribution: Normal<f64>,
}

impl NoiseSource {
    pub fn new(mean: f64, std_dev: f64) -> Result<Self, ModelError> {
        let distribution =
            Normal::new(mean, std_dev).map_err(|e| ModelError::Other(e.to_string()))?;
        if std_dev < 0.0 {
            return Err(ModelError::ConfigError(
                "Noise stdev cannot be negative".into(),
            ));
        }

        Ok(Self {
            mean,
            std_dev,
            distribution,
        })
    }
    pub fn new_zero_mean(std_dev: f64) -> Result<Self, ModelError> {
        let zero_mean = 0.0;
        Self::new(zero_mean, std_dev)
    }

    pub fn add_noise(&self, sample: DVector<f64>) -> DVector<f64> {
        if self.std_dev == 0.0 && self.mean == 0.0 {
            return sample;
        }
        let noise: DVector<f64> = DVector::from_fn(sample.len(), |_, _| {
            self.distribution.sample(&mut rand::thread_rng())
        });
        sample + noise
    }
}

pub struct NoiseSources(Vec<NoiseSource>);

impl NoiseSources {
    pub fn from_stats(noise_statistics: Vec<(f64, f64)>) -> Result<NoiseSources, ModelError> {
        let sources: Vec<NoiseSource> = noise_statistics
            .into_iter()
            .map(|(mean, std_dev)| NoiseSource::new(mean, std_dev))
            .collect::<Result<Vec<_>, ModelError>>()?;
        Ok(NoiseSources(sources))
    }

    pub fn add_noise(
        &self,
        source_idx: usize,
        sample: DVector<f64>,
    ) -> Result<DVector<f64>, ModelError> {
        if let Some(source) = self.0.get(source_idx) {
            return Ok(source.add_noise(sample));
        }

        Err(ModelError::Other(format!(
            "Attempted to access noise source {}, but there are only {} available",
            source_idx,
            self.0.len()
        )))
    }
}
