#[macro_export]
macro_rules! insert_scalar {
    ($var:expr, $registry:expr) => {
        $registry.insert_scalar(stringify!($var), ExprScalar::new($var.to_string()))
    };
}

#[macro_export]
macro_rules! insert_vector {
    ($var:expr, $registry:expr) => {
        $registry.insert_vector(stringify!($var), ExprVector::new($var))
    };
}
