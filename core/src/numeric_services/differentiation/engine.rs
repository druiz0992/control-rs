use super::error::DerivativeError;
use super::dtos::{DerivativeRequest, DerivativeResponse};

/// Specification of a derivative engine service
pub trait DerivativeEngine {
    ///   Computes derivatives for the given request. The request
    ///   specifies the functions, variables, and types of derivatives to compute (e.g., gradient,
    ///   Jacobian, Hessian). The response contains the computed derivatives or an error if the
    ///   computation fails.
    fn compute_derivatives(
        &self,
        req: &DerivativeRequest,
    ) -> Result<DerivativeResponse, DerivativeError>;
}
