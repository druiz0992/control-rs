use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

const FFI_FILE_STR: &str = "ffi.rs";
const GEN_DIR_STR: &str = "generated";
const SRC_DIR_STR: &str = "src";
const C_EXT_STR: &str = "c";

fn main() {
    let mut c_files = Vec::new();
    let mut include_dirs = HashSet::new();

    let project_root = Path::new(SRC_DIR_STR);
    find_all_generated_c_files(project_root, &mut c_files, &mut include_dirs);

    if c_files.is_empty() {
        println!("cargo:warning=No generated C files found. Skipping C compilation.");
        return;
    }

    println!("cargo:warning=Found {} generated C files", c_files.len());

    let mut build = cc::Build::new();
    build.files(&c_files);

    for dir in &include_dirs {
        build.include(dir);
    }

    build
        .flag_if_supported("-Wno-unused-variable")
        .compile("autogen");

    for file in c_files {
        println!("cargo:rerun-if-changed={}", file.display());
    }
}

fn find_all_generated_c_files(
    dir: &Path,
    c_files: &mut Vec<PathBuf>,
    include_dirs: &mut HashSet<PathBuf>,
) {
    if let Ok(entries) = fs::read_dir(dir) {
        let mut has_ffi = false;
        let mut generated_path = None;

        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() && path.file_name().is_some_and(|f| f == FFI_FILE_STR) {
                has_ffi = true;
            }
            if path.is_dir() && path.file_name().is_some_and(|f| f == GEN_DIR_STR) {
                generated_path = Some(path);
            }
        }

        if has_ffi {
            if let Some(gen_path) = generated_path {
                if let Ok(files) = fs::read_dir(&gen_path) {
                    for file in files.flatten() {
                        let path = file.path();
                        if path.extension().is_some_and(|ext| ext == C_EXT_STR) {
                            c_files.push(path.clone());
                            if let Some(parent) = path.parent() {
                                include_dirs.insert(parent.to_path_buf());
                            }
                        }
                    }
                }
            }
        }

        // Recurse into subdirectories
        for entry in fs::read_dir(dir).unwrap().flatten() {
            let path = entry.path();
            if path.is_dir() {
                find_all_generated_c_files(&path, c_files, include_dirs);
            }
        }
    }
}
