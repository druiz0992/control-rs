use control_rs::numeric_services::symbolic::ExprRegistry;
use control_rs::numeric_services::symbolic::fasteval::slab::ExprSlab;
use control_rs::physics::constants as c;
use control_rs::physics::models::{DoublePendulum, DoublePendulumInput, DoublePendulumState};
use control_rs::physics::traits::SymbolicDynamics;
use control_rs::utils::Labelizable;
use fasteval::compiler::IC;
use fasteval::slab::CompileSlab;
use fasteval::{Evaler, ExpressionI, Slab};
use fasteval::{Instruction, InstructionI};
use std::collections::HashMap;
use std::sync::Arc;

fn main() {
    let registry = Arc::new(ExprRegistry::new());

    let pendulum = DoublePendulum::new(1.0, 2.0, 1.0, 1.0, 1.0, Some(&registry));
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let dynamics_expr = pendulum.dynamics_symbolic(&state_symbol, &registry);

    let states = DoublePendulumState::labels();
    let actions = DoublePendulumInput::labels();
    let mut labels = Vec::with_capacity(states.len() + actions.len());

    labels.extend_from_slice(states);
    labels.extend_from_slice(actions);

    let slab = dynamics_expr.get_slab().unwrap();
    let code = generate_fn_from_slab(&slab, &labels);

    dbg!(&code);
}

fn build_hash(labels: &[&str]) -> HashMap<String, usize> {
    labels
        .iter()
        .enumerate()
        .map(|(i, s)| (s.to_string(), i))
        .collect()
}

fn generate_fn_from_slab(slabs: &ExprSlab, labels: &[&str]) -> String {
    match slabs {
        ExprSlab::Scalar(slab) => generate_fn_from_slab_scalar(slab, labels),
        ExprSlab::Vector(slabs) => generate_fn_from_slab_vec(slabs, labels),
        ExprSlab::Matrix(slabs) => generate_fn_from_slab_matrix(slabs, labels),
    }
}

fn process_parse_slab(
    slab: &Slab,
    r: &HashMap<String, usize>,
    code: &mut String,
    idx: &mut usize,
) -> bool {
    let Slab { cs, ps } = slab;
    let mut flag = false;
    match cs.get_instr(InstructionI(0)) {
        Instruction::IConst(c) if c.is_nan() => {
            let name = ps.get_expr(ExpressionI(0)).var_names(slab);
            if let Some(var_name) = name.first() {
                if let Some(i) = r.get(var_name) {
                    code.push_str(&format!("    let t{idx} = vals[{}];\n", i));
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

fn process_compile_slab(
    cs: &CompileSlab,
    r: &HashMap<String, usize>,
    code: &mut String,
    idx: &mut usize,
) {
    loop {
        let instr = cs.get_instr(InstructionI(*idx));
        match instr {
            Instruction::IConst(c) if c.is_nan() => break,
            Instruction::IVar(name) => {
                if let Some(i) = r.get(name) {
                    code.push_str(&format!("    let t{idx} = vals[{}];\n", i));
                } else {
                    panic!("Variable {} doesnt exist!", name);
                }
            }
            Instruction::INeg(a) => {
                code.push_str(&format!("    let t{idx} = -t{}\n", a.0));
            }
            Instruction::IAdd(a, IC::I(b)) => {
                code.push_str(&format!("    let t{idx} = t{} + t{};\n", a.0, b.0));
            }
            Instruction::IAdd(a, IC::C(c)) => {
                code.push_str(&format!("    let t{idx} = t{} + {:.8};\n", a.0, c));
            }
            Instruction::IMul(a, IC::I(b)) => {
                code.push_str(&format!("    let t{idx} = t{} * t{};\n", a.0, b.0));
            }
            Instruction::IMul(a, IC::C(c)) => {
                code.push_str(&format!("    let t{idx} = t{} * {:.8};\n", a.0, c));
            }
            Instruction::IFuncLog { base, of } => {
                let base_str = match base {
                    IC::I(i) => format!("t{}", i.0),
                    IC::C(c) => format!("{}", c),
                };
                let of_str = match of {
                    IC::I(i) => format!("t{}", i.0),
                    IC::C(c) => format!("{}", c),
                };
                code.push_str(&format!("    let t{idx} = {}.log({});\n", of_str, base_str));
            }
            Instruction::IExp { base, power } => {
                let base_str = match base {
                    IC::I(i) => format!("t{}", i.0),
                    IC::C(c) => format!("{}", c),
                };
                let power_str = match power {
                    IC::I(i) => format!("t{}", i.0),
                    IC::C(c) => format!("{}", c),
                };
                code.push_str(&format!(
                    "    let t{idx} = {}.powf({});\n",
                    base_str, power_str
                ));
            }

            Instruction::IInv(a) => {
                code.push_str(&format!("    let t{idx} = 1.0 / t{};\n", a.0));
            }
            Instruction::IFuncSin(a) => {
                code.push_str(&format!("    let t{idx} = t{}.sin();\n", a.0));
            }
            Instruction::IFuncCos(a) => {
                code.push_str(&format!("    let t{idx} = t{}.cos();\n", a.0));
            }
            Instruction::IFuncTan(a) => {
                code.push_str(&format!("    let t{idx} = t{}.tan();\n", a.0));
            }
            Instruction::IFuncAbs(a) => {
                code.push_str(&format!("    let t{idx} = t{}.abs();\n", a.0));
            }
            // ... handle other instructions
            _ => panic!("Unsupported instruction {:?}", instr),
        }
        *idx += 1;
    }
}

fn generate_fn_from_slab_scalar(slab: &Slab, labels: &[&str]) -> String {
    let mut code = String::new();
    let mut idx = 0;
    let r = build_hash(labels);

    code.push_str("fn eval(vals: &f64) -> Vec<f64> {\n");

    if process_parse_slab(slab, &r, &mut code, &mut idx) {
        code.push_str(&format!("    t{idx}\n"));
        return code;
    }

    let cs = &slab.cs;
    process_compile_slab(cs, &r, &mut code, &mut idx);

    code.push_str(&format!("    t{}\n", idx - 1)); // assuming last valid index is result
    code.push_str("}\n");

    code
}

fn generate_fn_from_slab_vec(slabs: &Vec<Slab>, labels: &[&str]) -> String {
    let mut code = String::new();
    let mut idx = 0;
    let mut v_idx = 0;
    let r = build_hash(labels);

    code.push_str("fn eval(vals: &[f64]) -> Vec<f64> {\n");
    code.push_str(&format!("    let mut v = vec![0.0; {}];\n", slabs.len()));

    for slab in slabs {
        if process_parse_slab(slab, &r, &mut code, &mut idx) {
            code.push_str(&format!("    v[{v_idx}] = t{idx};\n"));
            continue;
        }
        let cs = &slab.cs;
        process_compile_slab(cs, &r, &mut code, &mut idx);
        code.push_str(&format!("   v[{v_idx}] = t{};\n", idx - 1)); // assuming last valid index is result
        v_idx += 1;
    }
    code.push_str("    v\n"); // assuming last valid index is result
    code.push_str("}\n");

    code
}

fn generate_fn_from_slab_matrix(slabs: &Vec<Vec<Slab>>, labels: &[&str]) -> String {
    let mut code = String::new();
    let mut idx = 0;
    let mut r_idx = 0;
    let mut c_idx = 0;
    let r = build_hash(labels);

    code.push_str("fn eval(vals: &[Vec<f64>]) -> Vec<Vec<f64>> {\n");
    code.push_str(&format!(
        "    let mut v = vec![vec![0.0;{}];{}];\n",
        slabs[0].len(),
        slabs.len()
    ));

    for row in slabs {
        for slab in row {
            if process_parse_slab(slab, &r, &mut code, &mut idx) {
                code.push_str(&format!("    v[{r_idx}][{c_idx}] = t{idx};\n"));
                continue;
            }
            let cs = &slab.cs;
            process_compile_slab(cs, &r, &mut code, &mut idx);
            code.push_str(&format!("   v[{r_idx}][{c_idx}] = t{};\n", idx - 1)); // assuming last valid index is result
            c_idx += 1;
        }
        r_idx += 1;
    }
    code.push_str("    v\n"); // assuming last valid index is result
    code.push_str("}\n");

    code
}
