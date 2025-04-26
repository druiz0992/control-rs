
use std::process::{Command, Stdio};

pub fn display(filename: &str) -> std::io::Result<()> {
    Command::new("eog")
        .arg(filename)
        .stdout(Stdio::null()) // Redirect stdout to null
        .stderr(Stdio::null()) // Redirect stderr to null
        .spawn()?; // Spawn and don't wait

    Ok(())
}