pub trait Labelizable {
    fn labels() -> &'static [&'static str];
    fn index_of(label: &str) -> usize;
    fn vectorize(&self, labels: &[&str]) -> Vec<f64>;
    fn extract<const K: usize>(&self, labels: &[&str]) -> [f64; K];
}
