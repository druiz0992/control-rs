use super::dtos::CodegenRequest;
use super::error::CodegenError;

pub trait CodegenEngine {
    fn numerify(&self, req: &CodegenRequest) -> Result<(), CodegenError>;
}
