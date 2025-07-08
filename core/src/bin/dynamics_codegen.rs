use control_rs::physics::discretizer::{CodeGenerator, RK4Symbolic};
use control_rs::physics::models::{CartPole, DoublePendulum, Quadrotor2D};
use std::env;
use std::process::Command;
use std::sync::Arc;
use symbolic_services::symbolic::ExprRegistry;

/// # Run all (no args)
/// cargo run -p control-rs --bin dynamics_codegen
///
/// # Only double pendulum
/// cargo run -p control-rs --bin dynamics_codegen double_pendulum
///
/// # Only quadrotor
/// cargo run -p control-rs --bin dynamics_codegen quadrotor_2d
fn build_rk4_double_pendulum() {
    println!("Building code for RK4 Double Pendulum...");
    let registry = Arc::new(ExprRegistry::new());
    let (m1, m2, l1, l2, air_resistance_coeff) = (1.0, 1.0, 1.0, 1.0, 0.1);

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();

    integrator.to_numeric_jacobian_x().unwrap();
    integrator.to_numeric_jacobian_u().unwrap();
    integrator.to_numeric_da().unwrap();
    integrator.to_numeric_db().unwrap();
}

fn build_rk4_cart_pole() {
    println!("Building code for RK4 Cart Pole...");
    let registry = Arc::new(ExprRegistry::new());
    let (pole_mass, cart_mass, l, friction_coeff, air_resistance_coeff) = (1.0, 1.0, 1.0, 0.1, 0.1);

    let model = CartPole::new(
        pole_mass,
        cart_mass,
        l,
        friction_coeff,
        air_resistance_coeff,
        Some(&registry),
    );
    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();

    integrator.to_numeric_jacobian_x().unwrap();
    integrator.to_numeric_jacobian_u().unwrap();
    integrator.to_numeric_da().unwrap();
    integrator.to_numeric_db().unwrap();
}

fn build_rk4_quadrotor_2d() {
    println!("Building code for RK4 Quadrotor 2D...");
    let registry = Arc::new(ExprRegistry::new());
    let (m, j, l) = (1.0, 1.0, 1.0);

    let model = Quadrotor2D::new(m, j, l, Some(&registry));
    let integrator = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();

    integrator.to_numeric_jacobian_x().unwrap();
    integrator.to_numeric_jacobian_u().unwrap();
    integrator.to_numeric_da().unwrap();
    integrator.to_numeric_db().unwrap();
}

fn clean_ffi_codegen() {
    println!("Clean FFIs...");
    let status = Command::new("cargo")
        .args(["clean", "-p", "ffi_codegen"])
        .status()
        .expect("Failed to clean ffi_codegen crate");

    if !status.success() {
        panic!("❌ Failed to build ffi_codegen after code generation");
    } else {
        println!("✅ ffi_codegen built successfully");
    }
}
fn build_ffi_codegen() {
    println!("Building FFIs...");
    let status = Command::new("cargo")
        .args(["build", "-p", "ffi_codegen"])
        .status()
        .expect("Failed to build ffi_codegen crate");

    if !status.success() {
        panic!("❌ Failed to build ffi_codegen after code generation");
    } else {
        println!("✅ ffi_codegen built successfully");
    }
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();

    if args.contains(&"--help".to_string()) {
        println!("Usage: dynamics_codegen [model_name ...]");
        println!("Available models: double_pendulum, cart_pole, quadrotor_2d");
        return;
    }
    if args.is_empty() || args.contains(&"quadrotor_2d".to_string()) {
        build_rk4_quadrotor_2d();
    }

    if args.is_empty() || args.contains(&"cart_pole".to_string()) {
        build_rk4_cart_pole();
    }

    if args.is_empty() || args.contains(&"double_pendulum".to_string()) {
        build_rk4_double_pendulum();
    }

    clean_ffi_codegen();
    build_ffi_codegen();
}
