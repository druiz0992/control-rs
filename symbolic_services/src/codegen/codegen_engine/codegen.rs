use crate::codegen::dtos::CodegenRequest;
use crate::codegen::engine::CodegenEngine;
use crate::codegen::error::CodegenError;
use crate::python_client::PythonClient;

/// # Constants
/// - `PYTHON_SCRIPT_PATH`: Path to the Python script that performs the code generation
const PYTHON_SCRIPT_PATH: &str = "src/codegen/codegen_engine/codegen.py";

impl CodegenEngine for PythonClient {
    fn numerify(&self, req: &CodegenRequest) -> Result<(), CodegenError> {
        let mut path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        if !path.exists() {
            return Err(CodegenError::NotFound);
        }
        path.push(PYTHON_SCRIPT_PATH);

        self.run_python_json(path.to_str().unwrap(), req)
            .map_err(CodegenError::Other)?;

        Ok(())
    }
}
