use serde::{Deserialize, Serialize};

use crate::symbolic::{ExprRecord, ExprVector};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CodegenRequest {
    pub expr: ExprRecord,
    pub args: ExprVector,
    pub func_name: String,
    pub out_dir: String,
    pub mod_name: String,
    pub parallel: bool,
}

impl CodegenRequest {
    pub fn new(
        expr: ExprRecord,
        args: ExprVector,
        func_name: &str,
        out_dir: &str,
        mod_name: &str,
        parallel: bool,
    ) -> Self {
        Self {
            expr,
            args,
            func_name: func_name.into(),
            out_dir: out_dir.into(),
            mod_name: mod_name.into(),
            parallel,
        }
    }
}
