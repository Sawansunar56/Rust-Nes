use std::ops::Add;

struct CPU {
    a: u8,
    x: u8,
    y: u8,
    stat_reg: u8,
    sp: u8,
    pc: u16,

    memory: [u8; 0xffff],
}

enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    IndirectX,
    IndirectY,
    NoneAddressing,
}

impl CPU {
    fn new() -> Self {
        CPU {
            a: 0,
            sp: 0,
            pc: 0,
            x: 0,
            y: 0,
            stat_reg: 0,
            memory: todo!(),
        }
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&mut self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }
    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }

    fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.stat_reg = 0;

        self.pc = self.mem_read_u16(0xfffc);
    }

    fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xfffc, 0x8000);
    }

    fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    fn run(&mut self) {
        loop {
            let opcode = self.mem_read(self.pc);
            self.pc += 1;

            match opcode {}
        }
    }

    fn lda(&mut self, value: u8) {
        self.a = value;
        self.update_zero_and_negative_flags(self.a);
    }

    fn tax(&mut self) {
        self.x = self.a;
        self.update_zero_and_negative_flags(self.x);
    }

    fn inx(&mut self) {
        self.x += 1;
        self.update_zero_and_negative_flags(self.x);
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.stat_reg = self.stat_reg | 0b00000010;
        } else {
            self.stat_reg = self.stat_reg & 0b11111101;
        }

        if result & 0b1000000 != 0 {
            self.stat_reg = self.stat_reg | 0b10000000;
        } else {
            self.stat_reg = self.stat_reg & 0b01111111;
        }
    }
    fn interpret(&mut self, program: Vec<u8>) {
        self.pc = 0;

        loop {
            let cur_op = program[self.pc as usize];
            self.pc += 1;

            match cur_op {
                0x00 => return,
                0xA9 => {
                    // this is LDA
                    let params = program[self.pc as usize];
                    self.pc += 1;
                    self.lda(params);
                }
                0xAA => self.tax(),
                0xE8 => self.inx(),
                _ => todo!(),
            }
        }
    }


    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.pc,
            AddressingMode::ZeroPage => self.mem_read(self.pc) as u16,
            AddressingMode::Absolute => self.mem_read_u16(self.pc),
            AddressingMode::ZeroPageX => {
                let pos = self.mem_read(self.pc);
                let addr = pos.wrapping_add(self.x) as u16;
                addr
            }
            AddressingMode::ZeroPageY => {
                let pos = self.mem_read(self.pc);
                let addr = pos.wrapping_add(self.y) as u16;
                addr
            }
            AddressingMode::AbsoluteX => {
                let base = self.mem_read_u16(self.pc);
                let addr = base.wrapping_add(self.x as u16);
                addr
            }
            AddressingMode::AbsoluteY => {
                let base = self.mem_read_u16(self.pc);
                let addr = base.wrapping_add(self.y as u16);
                addr
            }
            AddressingMode::IndirectX => {
                let base = self.mem_read(self.pc);

                let ptr: u8 = (base as u8).wrapping_add(self.x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::IndirectY => {
                let base = self.mem_read(self.pc);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.y as u16);
                deref
            }
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} does not exist", mode);
            }
        }
    }
}

fn main() {
    let _cpu = CPU::new();
    println!("Hello, world!");
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_lda_0xa9_op_immediate_load() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.a, 0x05);
        assert!(cpu.stat_reg & 0b0000_0010 == 0b00);
        assert!(cpu.stat_reg & 0b1000_0000 == 0);
    }

    #[test]
    fn test_lda_0xa9_zero_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.stat_reg & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_tax_0xaa() {
        let mut cpu = CPU::new();
        cpu.a = 10;
        cpu.interpret(vec![0xaa, 0x00]);

        assert_eq!(cpu.x, 10);
    }

    #[test]
    fn test_inx_0xe8_increment_to_x() {
        let mut cpu = CPU::new();
        cpu.x = 10;
        cpu.interpret(vec![0xe8, 0x00]);

        assert_eq!(cpu.x, 11);
    }

    #[test]
    fn test_everything_before_this() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.x, 0xc1);
    }
}
