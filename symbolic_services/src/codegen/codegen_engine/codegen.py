import os
import json
import sys
import re
from sympy import (
    Symbol, symbols, sin, cos, tan, exp, log, sinh, cosh, tanh,
    Pow, sqrt, cse
)
from sympy.parsing.sympy_parser import (
    parse_expr, standard_transformations, implicit_multiplication_application
)
from sympy.printing import ccode



def build_c_function(expr_str, var_names, func_name):
    """    Build a C function from a symbolic expression."""
    vars = symbols(var_names)
    symbol_map = dict(zip(var_names, vars))

    function_map = {
        "sin": sin, "cos": cos, "tan": tan,
        "exp": exp, "log": log, "sqrt": sqrt, "pow": Pow,
        "sinh": sinh, "cosh": cosh, "tanh": tanh,
    }

    local_dict = {**symbol_map, **function_map}
    expr_str = expr_str.replace("^", "**")
    expr = parse_expr(
        expr_str,
        local_dict=local_dict,
        transformations=(standard_transformations + (implicit_multiplication_application,))
    )

    replacements, reduced_exprs = cse(expr)

    prototype = f"double {func_name}(double* args);"

    lines = []
    lines.append(f"double {func_name}(double* args) {{")
    for i, name in enumerate(var_names):
        lines.append(f"    double {name} = args[{i}];")
    lines.append("")
    for var, subexpr in replacements:
        lines.append(f"    double {ccode(var)} = {ccode(subexpr)};")
    lines.append(f"    return {ccode(reduced_exprs[0])};")
    lines.append("}")
    return prototype, "\n".join(lines)

def build_c_h_files(all_funcs, all_prototypes, func_name, out_dir):
    # Ensure output dir exists
    os.makedirs(out_dir, exist_ok=True)

    # Write the .c file
    c_path = os.path.join(out_dir, f"{func_name}.c")
    with open(c_path, "w") as f:
        f.write("#include <math.h>\n")
        f.write(f'#include "{func_name}.h"\n\n')
        for func_code in all_funcs:
            f.write(func_code)
            f.write("\n\n")

    # Write the .h file
    h_path = os.path.join(out_dir, f"{func_name}.h")
    guard = f"{func_name.upper()}H"
    with open(h_path, "w") as f:
        f.write(f"#ifndef {guard}\n")
        f.write(f"#define {guard}\n\n")
        for proto in all_prototypes:
            f.write(proto + "\n")
        f.write("\n#endif\n")


def build_ffi_header(func_names, out_dir):
    """Builds or updates the Rust FFI header file for generated C functions."""
    os.makedirs(out_dir, exist_ok=True)
    ffi_path = os.path.join(out_dir, f"ffi.rs")

    existing_lines = set()
    if os.path.exists(ffi_path):
        with open(ffi_path, "r") as f:
            existing_lines = set(line.strip() for line in f if line.strip())

    new_lines = []
    for ffi_func in func_names:
        line = f'pub (crate) fn {ffi_func}(args: *const f64) -> f64;'
        if line not in existing_lines:
            new_lines.append(f'    {line}\n')

    if new_lines:
        if 'unsafe extern "C" {' not in existing_lines:
            header = 'unsafe extern "C" {\n'
            footer = '}\n'
            content = header + ''.join(new_lines) + footer
            with open(ffi_path, "w") as f:
                f.write(content)
        else:
            # Append inside existing `extern` block
            with open(ffi_path, "r") as f:
                lines = f.readlines()

            # Insert before closing brace
            with open(ffi_path, "w") as f:
                for line in lines:
                    if line.strip() == "}":
                        f.writelines(new_lines)
                    f.write(line)



def build_codegen_export(func_name, all_func_names, mod_name,  out_dir):
    eval_fns = []

    os.makedirs(out_dir, exist_ok=True)
    for name in all_func_names:
        match = re.match(rf"{re.escape(func_name)}_(\d+)_(\d+)", name)
        if match:
            i, j = int(match.group(1)), int(match.group(2))
            eval_fns.append((name, i, j))  # remove .rs extension

    if not eval_fns:
        return  # Nothing to do

    m = max(i for _, i, _ in eval_fns) + 1
    n = max(j for _, _, j in eval_fns) + 1
    arity = n * m

    codegen_export_file = os.path.join(out_dir, "codegen_export.rs")
    fn_signature = f"pub fn {func_name}(args: &[f64])"

    # Check if function already exists
    if os.path.exists(codegen_export_file):
        with open(codegen_export_file, "r") as f:
            if any(fn_signature in line for line in f):
                return  # Function already written

    mod_name = mod_name.replace('/', '::')
    with open(codegen_export_file, "a") as f:
        if os.path.getsize(codegen_export_file) == 0:
            # Write headers only if file was empty
            f.write(f"use rayon::prelude::*;\nuse nalgebra::DMatrix;\nuse crate::{mod_name}::ffi::*;\n\n")

        f.write(f"{fn_signature} -> DMatrix<f64> {{\n")
        f.write(f"   let rows = {m};\n")
        f.write(f"   let cols = {n};\n\n")
        f.write("    let entries : [(\n")
        f.write("         unsafe extern \"C\" fn(*const f64) -> f64,\n")
        f.write("         usize,\n")
        f.write("         usize,\n")
        f.write(f"      );{arity}] = [\n")
        for name, i, j in eval_fns:
            f.write(f"        ({name} as unsafe extern \"C\" fn(*const f64) -> f64, {i},{j}),\n")
        f.write("    ];\n")
        f.write("    let mut mat = DMatrix::from_element(rows, cols, 0.0);\n")
        f.write("    let results: Vec<(usize, usize, f64)> = entries\n")
        f.write("        .par_iter()\n")
        f.write("        .map(|(f, i, j)| (*i, *j, unsafe {f(args.as_ptr()) }))\n")
        f.write("        .collect();\n")
        f.write("    for (i, j, val) in results {\n")
        f.write("        mat[(i,j)] = val;\n")
        f.write("    }\n")
        f.write("    mat\n}\n\n")

def normalize_expr(expr):
    """Convert expr dict to a 2D list of strings (matrix-like form)."""
    if isinstance(expr, dict):
        if "Matrix" in expr:
            return expr["Matrix"]
        elif "Vector" in expr:
            # Return as a single-row matrix
            return [expr["Vector"]]
        elif "Scalar" in expr:
            # Return as 1x1 matrix
            return [[expr["Scalar"]]]
        else:
            raise ValueError(f"Unknown expression format: {expr}")
    else:
        raise TypeError(f"Expected expr to be a dict, got {type(expr)}")

def ensure_mod_declaration(file_path, mod_name, is_pub=True):
    """Ensure `mod mod_name;` exists in the file."""
    mod_type = "pub " if mod_name.startswith("pub ") or is_pub else ""
    use_mod = "pub use " if mod_name.startswith("pub ") else ""
    mod_name = mod_name.replace("pub ", "").strip()
    line_to_add = f"{mod_type}mod {mod_name};\n"

    if not os.path.exists(file_path):
        with open(file_path, "w") as f:
            f.write(line_to_add)
    else:
        with open(file_path, "r") as f:
            lines = f.readlines()
        if not any(line.strip() == line_to_add.strip() for line in lines):
            with open(file_path, "a") as f:
                f.write(line_to_add)

    if use_mod == "pub use ":
        # if its pub mod, add a `pub use` line with all pub functions defined in the module
        use_line = f"pub use {mod_name}::*;\n"
        with open(file_path, "a") as f:
            if not any(line.strip() == use_line.strip() for line in lines):
                f.write(use_line)


def add_mods(mod_name, out_dir):
    parts = mod_name.split('/')
    top_mod = parts[0]

    # Ensure top-level mod declaration in lib.rs
    lib_file = os.path.join(out_dir, "lib.rs")
    ensure_mod_declaration(lib_file, top_mod)

    # Traverse and create nested mod.rs files
    current_path = out_dir
    for i, part in enumerate(parts[:-1]):
        current_path = os.path.join(current_path, part)
        os.makedirs(current_path, exist_ok=True)

        mod_file = os.path.join(current_path, "mod.rs")
        child = parts[i + 1]
        ensure_mod_declaration(mod_file, child)

    # Now handle the final level separately (inside the last part of the path)
    current_path = os.path.join(current_path, parts[-1])
    os.makedirs(current_path, exist_ok=True)
    mod_file = os.path.join(current_path, "mod.rs")

    # Create mod.rs if missing
    if not os.path.exists(mod_file):
        with open(mod_file, "w") as f:
            pass
    for mod_name in ["ffi", "pub codegen_export"]:
        ensure_mod_declaration(os.path.join(current_path, "mod.rs"), mod_name, is_pub=False)


if __name__ == "__main__":
    import sys, json

    input_data = sys.stdin.read()
    data = json.loads(input_data)

    expr = data['expr'] #.strip().strip('"')
    var_names = data['args']
    func_name = data['func_name']
    out_dir = data['out_dir']
    mod_name = data['mod_name']


    all_funcs = []
    all_prototypes = []
    all_func_names = []

    expr = normalize_expr(expr)
    for i, row in enumerate(expr):
        for j, cell in enumerate(row):
            fname = f"{func_name}_{i}_{j}"
            proto, code = build_c_function(cell, var_names, fname)
            all_funcs.append(code)
            all_prototypes.append(proto)
            all_func_names.append(fname)




    out_folder = os.path.join(out_dir, "src")
    mod_folder = os.path.join(out_folder, mod_name)
    c_folder = os.path.join(mod_folder, "generated")


    build_c_h_files(all_funcs, all_prototypes, func_name, c_folder)
    build_ffi_header(all_func_names, mod_folder)
    build_codegen_export(func_name, all_func_names, mod_name, mod_folder)
    add_mods(mod_name, out_folder)


    output_data = {
       'func_names': all_func_names
    }
    print(json.dumps(output_data))

