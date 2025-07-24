pub mod opcodes;
pub mod cpu;

fn main() {
    let mut cpu = cpu::CPU::new();

    cpu.run();
    println!("Hello, world!");
}

