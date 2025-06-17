use std::collections::{BTreeMap, HashMap};

use fasteval::{
    Evaler, Expression, ExpressionI, Instruction, InstructionI, Slab, ValueI, compiler::IC,
    slab::CompileSlab,
};
use nalgebra::DMatrix;
use regex::Regex;

use crate::{
    numeric_services::symbolic::{SymbolicError, SymbolicFunction, TryIntoEvalResult},
    utils::evaluable::Evaluable,
};

#[derive(Debug)]
pub enum InstructionSlab {
    Scalar((Instruction, Slab)),
    Vector(Vec<(Instruction, Slab)>),
    Matrix(Vec<Vec<(Instruction, Slab)>>),
}

impl InstructionSlab {
    pub fn generate_fn_from_slab(&self, function_name: &str, params: &[&str]) -> String {
        match self {
            InstructionSlab::Scalar(slab) => {
                generate_fn_from_slab_scalar(function_name, params, slab)
            }
            InstructionSlab::Vector(slabs) => {
                generate_fn_from_slab_vec(function_name, params, slabs)
            }
            InstructionSlab::Matrix(slabs) => {
                generate_fn_from_slab_matrix(function_name, params, slabs)
            }
        }
    }
}
fn generate_fn_from_slab_scalar(
    function_name: &str,
    params: &[&str],
    instruction_slab: &(Instruction, Slab),
) -> String {
    let mut idx = 0;
    let r = build_hash(params);

    let mut code = format!("/*\n");
    let instruction = &instruction_slab.0;
    let slab = &instruction_slab.1;
    code += format!("   {:?}\n", slab.cs).as_str();
    code += format!("\n\n\n     {:?}*/\n\n", slab).as_str();
    code.push_str(&format!("fn {}(params: &[f64]) -> f64 {{\n", function_name));

    if process_parsed_slab(slab, &r, &mut code, &mut idx) {
        code.push_str(&format!("    t{}\n", idx - 1));
        return code;
    }

    let cs = &slab.cs;
    process_compiled_instruction_slab(cs, instruction, &r, &mut code, &mut idx);

    code.push_str(&format!("    t{}\n", idx - 1)); // assuming last valid index is result
    code.push_str("}\n");

    insert_newlines_before_expression(&code)
}

fn build_hash(labels: &[&str]) -> HashMap<String, usize> {
    labels
        .iter()
        .enumerate()
        .map(|(i, s)| (s.to_string(), i))
        .collect()
}

fn process_parsed_slab(
    slab: &Slab,
    r: &HashMap<String, usize>,
    code: &mut String,
    idx: &mut usize,
) -> bool {
    let Slab { cs, ps } = slab;
    let mut flag = false;
    match cs.get_instr(InstructionI(0)) {
        Instruction::IConst(c) if c.is_nan() => {
            let expr = ps.get_expr(ExpressionI(0));
            let name = expr.var_names(slab);
            let mut map: BTreeMap<String, f64> = BTreeMap::new();
            if let Ok(val) = expr.eval(slab, &mut map) {
                code.push_str(&format!("    let t{idx} = {:.17e};\n", val));
                *idx += 1;
                flag = true;
            } else if let Some(var_name) = name.first() {
                if let Some(i) = r.get(var_name) {
                    code.push_str(&format!("    let t{idx} = params[{}];\n", i));
                    *idx += 1;
                    flag = true;
                } else {
                    panic!("Variable {} doesnt exist!", var_name);
                }
            } else {
                panic!("Variable");
            }
        }
        _ => (),
    };
    flag
}

fn process_compiled_instruction_slab(
    cs: &CompileSlab,
    instruction: &Instruction,
    r: &HashMap<String, usize>,
    code: &mut String,
    idx: &mut usize,
) {
    let mut instr_idx = 0;
    let start_idx = *idx;
    let mut break_flag = false;
    loop {
        let mut instr = cs.get_instr(InstructionI(instr_idx));
        instr = if let Instruction::IConst(c) = instr {
            if c.is_nan() && !break_flag {
                break_flag = true;
                instruction
            } else {
                instr
            }
        } else {
            instr
        };

        match instr {
            Instruction::IConst(c) if c.is_nan() => break,
            Instruction::IVar(name) => {
                if let Some(i) = r.get(name) {
                    code.push_str(&format!("    let t{idx} = params[{}];\n", i));
                } else {
                    panic!("Variable {} doesnt exist!", name);
                }
            }
            Instruction::INeg(a) => {
                code.push_str(&format!("    let t{idx} = -t{};\n", a.0 + start_idx));
            }
            Instruction::IAdd(a, IC::I(b)) => {
                code.push_str(&format!(
                    "    let t{idx} = t{} + t{};\n",
                    a.0 + start_idx,
                    b.0 + start_idx
                ));
            }
            Instruction::IAdd(a, IC::C(c)) => {
                code.push_str(&format!(
                    "    let t{idx} = t{} + {:.17e};\n",
                    a.0 + start_idx,
                    c
                ));
            }
            Instruction::IMul(a, IC::I(b)) => {
                code.push_str(&format!(
                    "    let t{idx} = t{} * t{};\n",
                    a.0 + start_idx,
                    b.0 + start_idx
                ));
            }
            Instruction::IMul(a, IC::C(c)) => {
                code.push_str(&format!(
                    "    let t{idx} = t{} * {:.17e};\n",
                    a.0 + start_idx,
                    c
                ));
            }
            Instruction::IFuncLog { base, of } => {
                let base_str = match base {
                    IC::I(i) => format!("t{}", i.0 + start_idx),
                    IC::C(c) => format!("{:.17e}", c),
                };
                let of_str = match of {
                    IC::I(i) => format!("t{}", i.0 + start_idx),
                    IC::C(c) => format!("{:.17e}", c),
                };
                code.push_str(&format!("    let t{idx} = {}.log({});\n", of_str, base_str));
            }
            Instruction::IExp { base, power } => {
                let base_str = match base {
                    IC::I(i) => format!("t{}", i.0 + start_idx),
                    IC::C(c) => format!("{:.17e}", c),
                };
                let power_str = match power {
                    IC::I(i) => format!("t{}", i.0 + start_idx),
                    IC::C(c) => format!("{:.17e}", c),
                };
                code.push_str(&format!(
                    "    let t{idx} = {}.powf({});\n",
                    base_str, power_str
                ));
            }

            Instruction::IInv(a) => {
                code.push_str(&format!(
                    "    let t{idx} = 1.0 / t{:.8};\n",
                    a.0 + start_idx
                ));
            }
            Instruction::IFuncSin(a) => {
                code.push_str(&format!("    let t{idx} = t{}.sin();\n", a.0 + start_idx));
            }
            Instruction::IFuncCos(a) => {
                code.push_str(&format!("    let t{idx} = t{}.cos();\n", a.0 + start_idx));
            }
            Instruction::IFuncTan(a) => {
                code.push_str(&format!("    let t{idx} = t{}.tan();\n", a.0 + start_idx));
            }
            Instruction::IFuncAbs(a) => {
                code.push_str(&format!("    let t{idx} = t{}.abs();\n", a.0 + start_idx));
            }
            // ... handle other instructions
            _ => panic!("Unsupported instruction {:?}", instr),
        }
        *idx += 1;
        instr_idx += 1;
    }
}

fn generate_fn_from_slab_vec(
    function_name: &str,
    params: &[&str],
    instruction_slabs: &Vec<(Instruction, Slab)>,
) -> String {
    let mut code = String::new();
    let mut idx = 0;
    let mut v_idx = 0;
    let r = build_hash(params);

    code.push_str(&format!(
        "fn {}(params: &[f64]) -> DVector<f64> {{\n",
        function_name
    ));
    code.push_str(&format!(
        "    let mut v:Vec<f64> = vec![0.0; {}];\n",
        instruction_slabs.len()
    ));

    for instruction_slab in instruction_slabs {
        let instruction = &instruction_slab.0;
        let slab = &instruction_slab.1;
        if process_parsed_slab(slab, &r, &mut code, &mut idx) {
            code.push_str(&format!("    v[{v_idx}] = t{};\n", idx - 1));
            v_idx += 1;
            continue;
        }
        let cs = &slab.cs;
        process_compiled_instruction_slab(cs, instruction, &r, &mut code, &mut idx);
        code.push_str(&format!("   v[{v_idx}] = t{};\n", idx - 1)); // assuming last valid index is result
        v_idx += 1;
    }
    code.push_str("    DVector::from_vec(v)\n"); // assuming last valid index is result
    code.push_str("}\n");

    insert_newlines_before_expression(&code)
}

fn insert_newlines_before_expression(input: &str) -> String {
    //let re = Regex::new(r#"(\d+:Expression)"#).unwrap();
    //re.replace_all(input, "\n$1").to_string()
    input.to_string()
}

fn generate_fn_from_slab_matrix(
    function_name: &str,
    params: &[&str],
    instruction_slabs: &Vec<Vec<(Instruction, Slab)>>,
) -> String {
    let mut idx = 0;
    let mut r_idx = 0;
    let mut c_idx = 0;
    let r = build_hash(params);

    let mut code = format!("/*\n");
    for row in instruction_slabs {
        for instruction_slab in row {
            code += format!("   {:?}\n", instruction_slab.1.cs).as_str();
        }
    }
    code += format!("\n\n\n     {:?}*/\n\n", instruction_slabs).as_str();
    code.push_str(&format!(
        "fn {}(params: &[f64]) -> DMatrix<f64> {{\n",
        function_name
    ));
    code.push_str(&format!(
        "    let mut v: Vec<Vec<f64>> = vec![vec![0.0;{}];{}];\n",
        instruction_slabs[0].len(),
        instruction_slabs.len()
    ));

    for row in instruction_slabs {
        for instruction_slab in row {
            let instruction = &instruction_slab.0;
            let slab = &instruction_slab.1;
            if process_parsed_slab(slab, &r, &mut code, &mut idx) {
                code.push_str(&format!("    v[{r_idx}][{c_idx}] = t{};\n", idx - 1));
                c_idx += 1;
                continue;
            }
            let cs = &slab.cs;
            process_compiled_instruction_slab(cs, instruction, &r, &mut code, &mut idx);
            code.push_str(&format!("   v[{r_idx}][{c_idx}] = t{};\n", idx - 1)); // assuming last valid index is result
            c_idx += 1;
        }
        r_idx += 1;
        c_idx = 0;
    }
    code.push_str("    vec_to_dmat(&v)\n"); // assuming last valid index is result
    code.push_str("}\n");

    insert_newlines_before_expression(&code)
}
