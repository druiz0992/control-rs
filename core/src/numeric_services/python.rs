use std::io::Write;
use std::process::{Command, Stdio};

/// # Constants
/// - `PYTHON_INTERPRETER`: The Python interpreter to use (default is `python3`).
const PYTHON_INTERPRETER: &str = "python3";

#[derive(Debug, Default)]
pub struct PythonClient {
    path: String,
}

impl PythonClient {
    pub fn new() -> Self {
        Self {
            path: PYTHON_INTERPRETER.to_string(),
        }
    }

    /// Changes python executable path
    pub fn set_python_path(&mut self, path: &str) {
        self.path = path.to_string();
    }

    pub fn run_python_json<T: serde::Serialize>(
        &self,
        script_path: &str,
        input: &T,
    ) -> Result<String, String> {
        let mut child = Command::new(self.path.as_str())
            .arg(script_path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|e| format!("Failed to start Python: {e}"))?;

        let json_input = serde_json::to_string(input)
            .map_err(|e| format!("Failed to serialize input JSON: {e}"))?;

        if let Some(stdin) = child.stdin.as_mut() {
            stdin
                .write_all(json_input.as_bytes())
                .map_err(|e| format!("Failed to write to stdin: {e}"))?;
        }

        let output = child
            .wait_with_output()
            .map_err(|e| format!("Failed to get output: {e}"))?;

        if !output.status.success() {
            return Err(format!(
                "Python script failed with: {}",
                String::from_utf8_lossy(&output.stderr)
            ));
        }

        Ok(String::from_utf8_lossy(&output.stdout).to_string())
    }
}
