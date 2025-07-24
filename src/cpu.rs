use std::collections::HashMap;

use crate::opcodes::{self, OpCode, OPCODES_MAP};

pub struct CPU {
    a: u8,
    x: u8,
    y: u8,
    stat_reg: u8,
    sp: u8,
    pc: u16,

    memory: [u8; 0xffff],
}

pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
    Accumulator,
    Relative,
    NoneAddressing,
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            a: 0,
            sp: 0,
            pc: 0,
            x: 0,
            y: 0,
            stat_reg: 0,
            memory: [0; 0xffff],
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

    pub fn run(&mut self) {
        let opcodes: &HashMap<u8, &'static OpCode> = &*OPCODES_MAP;

        loop {
            let cur_op = self.mem_read(self.pc);
            self.pc += 1;
            let pc_state = self.pc;

            let opcode = opcodes
                .get(&cur_op)
                .expect(&format!("OpCode {:x} is not recognized", cur_op));

            match cur_op {
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(&opcode.addressing_mode);
                }
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.addressing_mode);
                }
                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(&opcode.addressing_mode);
                }
                0x86 | 0x96 | 0x8E => {
                    self.stx(&opcode.addressing_mode);
                }
                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(&opcode.addressing_mode);
                }
                0x84 | 0x94 | 0x8C => {
                    self.sty(&opcode.addressing_mode);
                }
                0xAA => {
                    self.tax();
                }
                0x8A => {
                    self.txa();
                }
                0xA8 => {
                    self.tay();
                }
                0x98 => {
                    self.tya();
                }
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.addressing_mode);
                }
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.addressing_mode);
                }
                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(&opcode.addressing_mode);
                }
                0xC6 | 0xD6 | 0xCE | 0xDE => {
                    self.dec(&opcode.addressing_mode);
                }
                0xE8 => {
                    self.inx();
                }
                0xCA => {
                    self.dex();
                }
                0xC8 => {
                    self.iny();
                }
                0x88 => {
                    self.dey();
                }
                0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(&opcode.addressing_mode);
                }
                0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(&opcode.addressing_mode);
                }
                0x2A | 0x26 | 0x36 | 0x2E | 0x3E => {
                    self.rol(&opcode.addressing_mode);
                }
                0x6A | 0x66 | 0x76 | 0x6E | 0x7E => {
                    self.ror(&opcode.addressing_mode);
                }
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.addressing_mode);
                }
                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.addressing_mode);
                }
                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.addressing_mode);
                }
                0x24 | 0x2C => {
                    self.bit(&opcode.addressing_mode);
                }
                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.cmp(&opcode.addressing_mode);
                }
                0xE0 | 0xE4 | 0xEC => {
                    self.cpx(&opcode.addressing_mode);
                }
                0xC0 | 0xC4 | 0xCC => {
                    self.cpy(&opcode.addressing_mode);
                }
                0x90 => {
                    self.bcc();
                }
                0xB0 => {
                    self.bcs();
                }
                0xF0 => {
                    self.beq();
                }
                0xD0 => {
                    self.bne();
                }
                0x10 => {
                    self.bpl();
                }
                0x30 => {
                    self.bmi();
                }
                0x50 => {
                    self.bvc();
                }
                0x70 => {
                    self.bvs();
                }
                0x4C | 0x6C => {
                    self.jmp(&opcode.addressing_mode);
                }
                0x20 => {
                    self.jsr();
                }
                0x60 => {
                    self.rts();
                }
                0x40 => {
                    self.rti();
                }
                0x48 => {
                    self.pha();
                }
                0x68 => {
                    self.pla();
                }
                0x08 => {
                    self.php();
                }
                0x28 => {
                    self.plp();
                }
                0x9A => {
                    self.txs();
                }
                0xBA => {
                    self.tsx();
                }
                0x18 => {
                    self.clc();
                }
                0x38 => {
                    self.sec();
                }
                0x58 => {
                    self.cli();
                }
                0x78 => {
                    self.sei();
                }
                0xD8 => {
                    self.cld();
                }
                0xF8 => {
                    self.sed();
                }
                0xB8 => {
                    self.clv();
                }
                0xEA => {
                    self.nop();
                }

                _ => todo!(),
            }

            if pc_state == self.pc {
                self.pc += (opcode.bytes - 1) as u16;
            }
        }
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.a = value;
        self.update_zero_and_negative_flags(self.a);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.y = value;
        self.update_zero_and_negative_flags(self.y);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.y);
    }

    fn tax(&mut self) {
        self.x = self.a;
        self.update_zero_and_negative_flags(self.x);
    }

    fn txa(&mut self) {
        self.a = self.x;
        self.update_zero_and_negative_flags(self.a);
    }

    fn tay(&mut self) {
        self.y = self.a;
        self.update_zero_and_negative_flags(self.y);
    }

    fn tya(&mut self) {
        self.a = self.y;
        self.update_zero_and_negative_flags(self.a);
    }

    fn inx(&mut self) {
        self.x += 1;
        self.update_zero_and_negative_flags(self.x);
    }
    fn dex(&mut self) {
        self.x -= 1;
        self.update_zero_and_negative_flags(self.x);
    }
    fn iny(&mut self) {
        self.y += 1;
        self.update_zero_and_negative_flags(self.x);
    }
    fn dey(&mut self) {
        self.y -= 1;
        self.update_zero_and_negative_flags(self.x);
    }

    fn bcc(&mut self) {
        let offset = self.mem_read(self.pc + 1) as i8;
        let new_pc = self.pc.wrapping_add(2).wrapping_add(offset as u16);
    }
    fn bcs(&mut self) {
        todo!()
    }
    fn beq(&mut self) {
        todo!()
    }
    fn bne(&mut self) {
        todo!()
    }
    fn bpl(&mut self) {
        todo!()
    }
    fn bmi(&mut self) {
        todo!()
    }
    fn bvc(&mut self) {
        todo!()
    }
    fn bvs(&mut self) {
        todo!()
    }
    fn jsr(&mut self) {
        todo!()
    }
    fn rts(&mut self) {
        todo!()
    }
    fn rti(&mut self) {
        todo!()
    }
    fn pha(&mut self) {
        todo!()
    }
    fn pla(&mut self) {
        todo!()
    }
    fn php(&mut self) {
        todo!()
    }
    fn plp(&mut self) {
        todo!()
    }
    fn txs(&mut self) {
        todo!()
    }
    fn tsx(&mut self) {
        todo!()
    }
    fn clc(&mut self) {
        todo!()
    }
    fn sec(&mut self) {
        todo!()
    }
    fn cli(&mut self) {
        todo!()
    }
    fn sei(&mut self) {
        todo!()
    }
    fn cld(&mut self) {
        todo!()
    }
    fn sed(&mut self) {
        todo!()
    }
    fn clv(&mut self) {
        todo!()
    }
    fn nop(&mut self) {
        todo!()
    }
    fn asl(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn lsr(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn rol(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn ror(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.a = self.a & value;

        self.update_zero_and_negative_flags(self.a);
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.a = self.a | value;

        self.update_zero_and_negative_flags(self.a);
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.a = self.a ^ value;

        self.update_zero_and_negative_flags(self.a);
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.a & value;

        // update zero flag
        if result == 0 {
            self.stat_reg = self.stat_reg | 0b00000010;
        } else {
            self.stat_reg = self.stat_reg & 0b11111101;
        }

        // update overflow flag
        if value & 0b010_0000 != 0 {
            self.stat_reg = self.stat_reg | 0b0100_0000;
        } else {
            self.stat_reg = self.stat_reg & 0b1011_1111;
        }

        // update negative flag
        if value & 0b100_0000 != 0 {
            self.stat_reg = self.stat_reg | 0b1000_0000;
        } else {
            self.stat_reg = self.stat_reg & 0b0111_1111;
        }
    }

    fn cmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.a.wrapping_sub(value);

        if self.a >= value {
            self.stat_reg = self.stat_reg | 0b0000_0001;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0001;
        }

        if self.a == value {
            self.stat_reg = self.stat_reg | 0b0000_0010;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0010;
        }

        if result & 0b100_0000 != 0 {
            self.stat_reg = self.stat_reg | 0b1000_0000;
        } else {
            self.stat_reg = self.stat_reg & !0b1000_0000;
        }
    }

    fn cpx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.x.wrapping_sub(value);

        if self.x >= value {
            self.stat_reg = self.stat_reg | 0b0000_0001;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0001;
        }

        if self.x == value {
            self.stat_reg = self.stat_reg | 0b0000_0010;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0010;
        }

        if result & 0b100_0000 != 0 {
            self.stat_reg = self.stat_reg | 0b1000_0000;
        } else {
            self.stat_reg = self.stat_reg & !0b1000_0000;
        }
    }

    fn cpy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.y.wrapping_sub(value);

        if self.y >= value {
            self.stat_reg = self.stat_reg | 0b0000_0001;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0001;
        }

        if self.y == value {
            self.stat_reg = self.stat_reg | 0b0000_0010;
        } else {
            self.stat_reg = self.stat_reg & !0b0000_0010;
        }

        if result & 0b100_0000 != 0 {
            self.stat_reg = self.stat_reg | 0b1000_0000;
        } else {
            self.stat_reg = self.stat_reg & !0b1000_0000;
        }
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        todo!()
    }

    // This is a read-modify-write instruction,
    // meaning that it first writes the original value back to memory
    // before the modified value.
    // This extra write can matter if targeting a hardware register.
    // NOTE: I don't know if the above statement matters for emulation because
    // I am going to emulate the cycles manually as well.
    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.mem_write(addr, value + 1);

        self.update_zero_and_negative_flags(value + 1);
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.mem_write(addr, value - 1);

        self.update_zero_and_negative_flags(value - 1);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        todo!()
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.x = value;
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

    // fn interpret(&mut self, program: Vec<u8>) {
    //     self.pc = 0;
    //
    //     loop {
    //         let cur_op = program[self.pc as usize];
    //         self.pc += 1;
    //
    //         match cur_op {
    //             0x00 => return,
    //             0xA9 => {
    //                 // this is LDA
    //                 let params = program[self.pc as usize];
    //                 self.pc += 1;
    //                 self.lda(params);
    //             }
    //             0xAA => self.tax(),
    //             0xE8 => self.inx(),
    //             _ => todo!(),
    //         }
    //     }
    // }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
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
                panic!("mode does not exist");
            }
            _ => todo!(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_lda_0xa9_op_immediate_load() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.a, 0x05);
        assert!(cpu.stat_reg & 0b0000_0010 == 0b00);
        assert!(cpu.stat_reg & 0b1000_0000 == 0);
    }

    #[test]
    fn test_lda_0xa9_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.stat_reg & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_tax_0xaa() {
        let mut cpu = CPU::new();
        cpu.a = 10;
        cpu.load_and_run(vec![0xaa, 0x00]);

        assert_eq!(cpu.x, 10);
    }

    #[test]
    fn test_inx_0xe8_increment_to_x() {
        let mut cpu = CPU::new();
        cpu.x = 10;
        cpu.load_and_run(vec![0xe8, 0x00]);

        assert_eq!(cpu.x, 11);
    }

    #[test]
    fn test_everything_before_this() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.x, 0xc1);
    }

    #[test]
    fn test_lds_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.a, 0x55);
    }
}
