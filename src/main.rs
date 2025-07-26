pub mod cpu;
pub mod opcodes;

use crate::cpu::CPU;

fn main() {
    let mut cpu = CPU::new();

    cpu.load_and_run(vec![0xa9, 0x05, 0x00]);

    println!("Hello, world!");
}

