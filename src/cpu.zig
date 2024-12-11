const std = @import("std");
const log = std.log;

const WORD = u32;
const WORD_BIT_WIDTH_PLUS_ONE = u6;
const MEMORY_SIZE = 0x10000; // 64KiB
const MEMORY_BASE_ADDR = 0x80000000; // riscv-tests

pub const CPUError = error{
    TooLargeMemoryData,
    OutOfMemoryArea,
    IllegalInstruction,
    InvalidCSR,
    ReadOnlyCSR,
    InvalidAlignment,
    EcallInvoked,
};

pub const CPU = struct {
    pc: WORD,
    regs: [32]WORD,
    mem: [MEMORY_SIZE >> 2]WORD,

    // CSRs
    hardware_thread_id: u32 = 0,
    trap_handler_base_address: WORD = 0,

    const CSR_MHARTID: u12 = 0xF14; // Hardware thread ID
    const CSR_MTVEC: u12 = 0x305; // Machine trap handler base address
    const CSR_MNSTATUS: u12 = 0x744; // Resumale NMI status
    const CSR_SATP: u12 = 0x180; // Supervisor address translation and protection
    const CSR_PMPADDR0: u12 = 0x3B0; // Physical memory protection address register
    const CSR_PMPCFG0: u12 = 0x3A0; // Physical memory protection configuration
    const CSR_MIE: u12 = 0x304; // Machine interrupt enable register
    const CSR_MEDELEG: u12 = 0x302; // Machine execption delegation register
    const CSR_MIDELEG: u12 = 0x303; // Machine interrupt delegation register
    const CSR_MSTATUS: u12 = 0x300; // Machine status register
    const CSR_MEPC: u12 = 0x341; // Machine exception program counter

    const Self = @This();

    pub fn init() Self {
        return .{
            .pc = MEMORY_BASE_ADDR, // for riscv-tests
            .regs = [1]WORD{0} ** 32,
            .mem = [1]WORD{0} ** (MEMORY_SIZE >> 2),
        };
    }

    pub fn load_memory(self: *Self, data: []const u8, offset: WORD) CPUError!void {
        if (data.len > self.mem.len + offset) {
            return CPUError.TooLargeMemoryData;
        }
        var i: WORD = 0;
        while (i < data.len) {
            var word_i: WORD = 0;
            var val: WORD = 0;
            while (word_i < @sizeOf(WORD) and i + word_i < data.len) {
                val |= @as(WORD, data[offset + i + word_i]) << @intCast((8 * word_i));
                word_i += 1;
            }
            self.mem[offset + i / 4] = val;
            i += 4;
        }
    }

    pub fn tick_cycle(self: *Self) CPUError!void {
        if (self.pc > self.mem.len + MEMORY_BASE_ADDR) {
            return CPUError.OutOfMemoryArea;
        }
        if (self.pc & 0x3 != 0) {
            return CPUError.InvalidAlignment;
        }

        const inst_addr = self.pc - MEMORY_BASE_ADDR;

        // instructions are stored in 4 bytes aligned
        const inst = self.mem[inst_addr >> 2];
        log.debug("MEM[0x{x}] INST=0b{b} (0x{x})", .{ self.pc, inst, inst });
        var ecall_exit = false;
        errdefer {
            if (!ecall_exit) {
                std.log.err("Illegal instruction at MEM[0x{x}] INST=0b{b} (0x{x})", .{ self.pc, inst, inst });
            }
        }

        if (inst == 0) {
            return CPUError.IllegalInstruction;
        }
        const rd: u5 = @intCast((inst >> 7) & 0x1F); // rd is inst[11:7]
        const rs1: u5 = @intCast((inst >> 15) & 0x1F); // rs1 is inst[19:15]
        const rs2: u5 = @intCast((inst >> 20) & 0x1F); // rs2 is inst[24:20]
        const funct3 = (inst >> 12) & 0x7; // funct3 is inst[14:12]
        const funct7 = (inst >> 25); // funct7 is inst[31:25]
        switch (inst & 0x7F) {
            0b0000011 => {
                var offset = (inst >> 20);
                if (offset >> 11 == 1) {
                    offset |= 0xFFFF_F800;
                }
                const mem_addr = @addWithOverflow(self.read_reg(rs1), offset)[0];

                // TODO: remove MEMORY_BASE_ADDR
                const mem_real_addr = mem_addr - MEMORY_BASE_ADDR;
                var mem_val = self.mem[mem_real_addr >> 2];

                switch (funct3) {
                    0b000, 0b100 => {
                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => mem_val &= 0xFF,
                            0b01 => mem_val = (mem_val >> 8) & 0xFF,
                            0b10 => mem_val = (mem_val >> 16) & 0xFF,
                            0b11 => mem_val = (mem_val >> 24) & 0xFF,
                        }

                        if (funct3 == 0b000) {
                            log.debug("LB rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = sign_ext(mem_val, 8);
                        } else {
                            log.debug("LBU rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = mem_val;
                        }
                    },
                    0b001, 0b101 => {
                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => mem_val &= 0xFFFF,
                            0b01 => mem_val = (mem_val >> 8) & 0xFFFF,
                            0b10 => mem_val = (mem_val >> 16) & 0xFFFF,
                            0b11 => mem_val = ((self.mem[(mem_real_addr >> 2) + 1] & 0xFF) << 8) | ((mem_val >> 24) & 0xFF),
                        }

                        if (funct3 == 0b001) {
                            log.debug("LH rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = sign_ext(mem_val, 16);
                        } else {
                            log.debug("LHU rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = mem_val;
                        }
                    },
                    0b010 => {
                        log.debug("LW rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });

                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => {},
                            0b01 => mem_val = ((self.mem[(mem_real_addr >> 2) + 1] & 0xFF) << 24) | (mem_val >> 8),
                            0b10 => mem_val = ((self.mem[(mem_real_addr >> 2) + 1] & 0xFFFF) << 16) | (mem_val >> 16),
                            0b11 => mem_val = ((self.mem[(mem_real_addr >> 2) + 1] & 0xFFFFFF) << 8) | (mem_val >> 24),
                        }

                        self.regs[rd] = mem_val;
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b0100011 => {
                const offset = sign_ext((funct7 << 5) | rd, 12);
                const mem_addr = @addWithOverflow(self.read_reg(rs1), offset)[0];
                var rs2_val = self.read_reg(rs2);
                // TODO: remove MEMORY_BASE_ADDR
                const mem_real_addr = mem_addr - MEMORY_BASE_ADDR;
                const mem_val = self.mem[mem_real_addr >> 2];

                switch (funct3) {
                    0b000 => {
                        rs2_val &= 0xFF;
                        log.debug("SB rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });

                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => self.mem[mem_real_addr >> 2] = (mem_val & 0xFFFFFF00) | rs2_val,
                            0b01 => self.mem[mem_real_addr >> 2] = (mem_val & 0xFFFF00FF) | (rs2_val << 8),
                            0b10 => self.mem[mem_real_addr >> 2] = (mem_val & 0xFF00FFFF) | (rs2_val << 16),
                            0b11 => self.mem[mem_real_addr >> 2] = (mem_val & 0x00FFFFFF) | (rs2_val << 24),
                        }
                    },
                    0b001 => {
                        rs2_val &= 0xFFFF;
                        log.debug("SH rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });

                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => self.mem[mem_real_addr >> 2] = (mem_val & 0xFFFF0000) | rs2_val,
                            0b01 => self.mem[mem_real_addr >> 2] = (mem_val & 0xFF0000FF) | (rs2_val << 8),
                            0b10 => self.mem[mem_real_addr >> 2] = (mem_val & 0x0000FFFF) | (rs2_val << 16),
                            0b11 => {
                                self.mem[mem_real_addr >> 2] = (mem_val & 0x00FFFFFF) | (rs2_val << 24);
                                self.mem[(mem_real_addr >> 2) + 1] = (self.mem[(mem_real_addr >> 2) + 1] & 0xFFFFFF00) | (rs2_val >> 8);
                            },
                        }
                    },
                    0b010 => {
                        log.debug("SW rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });

                        switch (@as(u2, @intCast(mem_real_addr & 0x3))) {
                            0b00 => self.mem[mem_real_addr >> 2] = rs2_val,
                            0b01 => {
                                self.mem[mem_real_addr >> 2] = (mem_val & 0x000000FF) | (rs2_val << 8);
                                self.mem[(mem_real_addr >> 2) + 1] = (self.mem[(mem_real_addr >> 2) + 1] & 0xFFFFFF00) | (rs2_val >> 24);
                            },
                            0b10 => {
                                self.mem[mem_real_addr >> 2] = (mem_val & 0x0000FFFF) | (rs2_val << 16);
                                self.mem[(mem_real_addr >> 2) + 1] = (self.mem[(mem_real_addr >> 2) + 1] & 0xFFFF0000) | (rs2_val >> 16);
                            },
                            0b11 => {
                                self.mem[mem_real_addr >> 2] = (mem_val & 0x00FFFFFF) | (rs2_val << 24);
                                self.mem[(mem_real_addr >> 2) + 1] = (self.mem[(mem_real_addr >> 2) + 1] & 0xFF000000) | (rs2_val >> 8);
                            },
                        }
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b1101111 => {
                var imm20: WORD = 0;
                if (inst >> 31 == 1) {
                    imm20 = 0xFFF;
                }
                const imm10_1 = (inst >> 21) & 0x3FF;
                const imm11 = (inst >> 20) & 0x1;
                const imm19_12 = (inst >> 12) & 0xFF;
                const imm = (imm20 << 20) | (imm19_12 << 12) | (imm11 << 11) | (imm10_1 << 1);

                log.debug("JAL rd={} imm={}", .{ rd, imm });
                self.regs[rd] = @addWithOverflow(self.pc, 4)[0];
                self.pc = @addWithOverflow(self.pc, imm)[0]; // adding any values including negative one
                return;
            },
            0b1100111 => {
                if (funct3 != 0) {
                    return CPUError.IllegalInstruction;
                }
                var imm = (inst >> 20);
                if (imm >> 11 == 1) {
                    imm |= 0xFFFF_F800;
                }
                log.debug("JALR rd={} rs1={} imm={}", .{ rd, rs1, imm });
                const rs1_val = self.read_reg(rs1);
                self.regs[rd] = self.pc + 4;

                // LSB is set to zero.
                self.pc = @addWithOverflow(rs1_val, imm)[0] & (0xFFFFFFFE);
                return;
            },
            0b0110111 => {
                const imm = inst & 0xFFFF_F000;
                log.debug("LUI rd={} imm={}", .{ rd, imm });
                self.regs[rd] = imm;
            },
            0b0010111 => {
                const imm = inst & 0xFFFF_F000;
                log.debug("AUIPC rd={} imm={}", .{ rd, imm });
                self.regs[rd] = @addWithOverflow(self.pc, imm)[0];
            },
            0b0110011 => {
                const rs1_val = self.read_reg(rs1);
                const rs2_val = self.read_reg(rs2);
                switch (funct3) {
                    0b000 => {
                        if (funct7 == 0) {
                            log.debug("ADD rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                            self.regs[rd] = @addWithOverflow(rs1_val, rs2_val)[0];
                        } else if (funct7 == 0b0100000) {
                            log.debug("SUB rd={} rs1={} 0x{x} rs2={} 0x{x}", .{ rd, rs1, rs1_val, rs2, rs2_val });
                            self.regs[rd] = @subWithOverflow(rs1_val, rs2_val)[0];
                            log.debug("rs1_val={} rs2_val={} res={}", .{ @as(i32, @bitCast(rs1_val)), @as(i32, @bitCast(rs2_val)), @as(i32, @bitCast(self.regs[rd])) });
                        } else {
                            return CPUError.IllegalInstruction;
                        }
                    },
                    0b001 => {
                        if (funct7 != 0) {
                            return CPUError.IllegalInstruction;
                        }
                        log.debug("SLL rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        // rs1 by the shift amount held in the lower 5 bits of register rs2.
                        self.regs[rd] = std.math.shl(WORD, self.read_reg(rs1), self.read_reg(rs2) & 0x1F);
                    },
                    0b010 => {
                        if (funct7 != 0) {
                            return CPUError.IllegalInstruction;
                        }
                        log.debug("SLT rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        if (@as(i32, @bitCast(rs1_val)) < @as(i32, @bitCast(rs2_val))) {
                            self.regs[rd] = 1;
                        } else {
                            self.regs[rd] = 0;
                        }
                    },
                    0b011 => {
                        if (funct7 != 0) {
                            return CPUError.IllegalInstruction;
                        }
                        log.debug("SLTU rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        if (rs1_val < rs2_val) {
                            self.regs[rd] = 1;
                        } else {
                            self.regs[rd] = 0;
                        }
                    },
                    0b100 => {
                        if (funct7 != 0) {
                            return CPUError.IllegalInstruction;
                        }
                        log.debug("XOR rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        self.regs[rd] = rs1_val ^ rs2_val;
                    },
                    0b101 => {
                        const shamt: u5 = @intCast(rs2_val & 0x1F);
                        if (funct7 == 0b0000000) {
                            log.debug("SRL rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                            self.regs[rd] = rs1_val >> shamt;
                        } else if (funct7 == 0b0100000) {
                            log.debug("SRA rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                            const bit_width = @bitSizeOf(WORD) - @as(WORD_BIT_WIDTH_PLUS_ONE, shamt);
                            log.debug("shamt={} bit_width={}", .{ shamt, bit_width });
                            self.regs[rd] = sign_ext(rs1_val >> shamt, bit_width);
                        } else {
                            return CPUError.IllegalInstruction;
                        }
                    },
                    0b110 => {
                        log.debug("OR rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        self.regs[rd] = rs1_val | rs2_val;
                    },
                    0b111 => {
                        log.debug("AND rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                        self.regs[rd] = rs1_val & rs2_val;
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b0010011 => {
                const rs1_val = self.read_reg(rs1);
                const imm = sign_ext(inst >> 20, 12);
                switch (funct3) {
                    0b000 => {
                        log.debug("ADDI rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        self.regs[rd] = @addWithOverflow(rs1_val, imm)[0];
                        log.debug("rd_val= 0x{x}", .{self.read_reg(rd)});
                    },
                    0b001 => {
                        const shamt: u5 = @intCast(imm & 0x1F);
                        log.debug("SLLI rd={} rs1={} shamt={}", .{ rd, rs1, shamt });
                        self.regs[rd] = self.read_reg(rs1) << shamt;
                    },
                    0b010 => {
                        log.debug("SLTI rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        if (@as(i32, @bitCast(rs1_val)) < @as(i32, @bitCast(imm))) {
                            self.regs[rd] = 1;
                        } else {
                            self.regs[rd] = 0;
                        }
                    },
                    0b011 => {
                        log.debug("SLTIU rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        if (rs1_val < imm) {
                            self.regs[rd] = 1;
                        } else {
                            self.regs[rd] = 0;
                        }
                    },
                    0b100 => {
                        log.debug("XORI rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        self.regs[rd] = rs1_val ^ imm;
                    },
                    0b101 => {
                        const shamt: u5 = @intCast(imm & 0x1F);
                        switch (inst >> 26) {
                            0b000000 => {
                                log.debug("SRLI rd={} rs1={} shamt={}", .{ rd, rs1, shamt });
                                self.regs[rd] = self.read_reg(rs1) >> shamt;
                            },
                            0b010000 => {
                                log.debug("SRAI rd={} rs1={} shamt={}", .{ rd, rs1, shamt });
                                const bit_width = @bitSizeOf(WORD) - @as(WORD_BIT_WIDTH_PLUS_ONE, @intCast(shamt));
                                self.regs[rd] = sign_ext(self.read_reg(rs1) >> shamt, bit_width);
                            },
                            else => {
                                return CPUError.IllegalInstruction;
                            },
                        }
                    },
                    0b110 => {
                        log.debug("ORI rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        self.regs[rd] = self.read_reg(rs1) | imm;
                    },
                    0b111 => {
                        log.debug("ANDI rd={} rs1={} imm={}", .{ rd, rs1, imm });
                        self.regs[rd] = self.read_reg(rs1) & imm;
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b1100011 => {
                var imm_12: WORD = (inst >> 31);
                const imm_11 = (inst >> 7) & 0x1;
                const imm_10_5 = (inst >> 25) & 0x3F;
                const imm_4_1 = (inst >> 8) & 0xF;
                if (imm_12 == 1) {
                    imm_12 = 0xFFFF_F000;
                }
                const imm = imm_12 | (imm_11 << 11) | (imm_10_5 << 5) | (imm_4_1 << 1);
                switch (funct3) {
                    0b000 => {
                        log.debug("BEQ rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (self.read_reg(rs1) == self.read_reg(rs2)) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    0b001 => {
                        log.debug("BNE rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (self.read_reg(rs1) != self.read_reg(rs2)) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    0b100 => {
                        const rs1_val: i32 = @bitCast(self.read_reg(rs1));
                        const rs2_val: i32 = @bitCast(self.read_reg(rs2));
                        log.debug("BLT rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (rs1_val < rs2_val) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    0b101 => {
                        const rs1_val: i32 = @bitCast(self.read_reg(rs1));
                        const rs2_val: i32 = @bitCast(self.read_reg(rs2));
                        log.debug("BGE rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (rs1_val >= rs2_val) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    0b110 => {
                        log.debug("BLTU rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (self.read_reg(rs1) < self.read_reg(rs2)) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    0b111 => {
                        log.debug("BGEU rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        if (self.read_reg(rs1) >= self.read_reg(rs2)) {
                            self.pc = @addWithOverflow(self.pc, imm)[0];
                            return;
                        }
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b0001111 => {
                switch (funct3) {
                    0b000 => {
                        log.debug("FENCE", .{});
                        log.debug("TODO: implement FENCE", .{});
                    },
                    0b001 => {
                        log.debug("FENCE.I", .{});
                        log.debug("TODO: implement FENCE.I", .{});
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
            },
            0b1110011 => {
                const csr: u12 = @intCast(inst >> 20);
                if (inst == 0x73) {
                    log.debug("ECALL", .{});
                    ecall_exit = true;
                    return CPUError.EcallInvoked;
                } else if (inst == 0x100073) {
                    log.debug("EBREAK", .{});
                    log.warn("TODO: implement EBREAK", .{});
                } else {
                    switch (funct3) {
                        0b000 => {
                            if (rd == 0 and rs1 == 0 and csr == 0b001100000010) {
                                log.debug("MRET", .{});
                                log.warn("TODO: implement MRET", .{});
                            }
                        },
                        0b001 => {
                            log.debug("CSRRW rd={} rs1={} csr=0x{x}", .{ rd, rs1, csr });
                            if (rd != 0) {
                                const csr_val = try self.read_csr(csr);
                                self.regs[rd] = csr_val;
                            }
                            try self.write_csr(csr, self.read_reg(rs1));
                        },
                        0b010 => {
                            log.debug("CSRRS rd={} rs1={} csr=0x{x}", .{ rd, rs1, csr });
                            const csr_val = try self.read_csr(csr);
                            self.regs[rd] = csr_val;
                            if (rs1 != 0) {
                                try self.write_csr(csr, csr_val | self.regs[rs1]);
                            }
                        },
                        0b101 => {
                            const uimm = rs1;
                            log.debug("CSRRWI rd={} uimm={} csr=0x{x}", .{ rd, uimm, csr });
                            if (rd != 0) {
                                const csr_val = try self.read_csr(csr);
                                self.regs[rd] = csr_val;
                            }
                            try self.write_csr(csr, @intCast(uimm));
                        },
                        else => {
                            return CPUError.IllegalInstruction;
                        },
                    }
                }
            },
            else => return CPUError.IllegalInstruction,
        }

        self.pc += 4;
    }

    pub fn read_reg(self: *Self, rd: u5) WORD {
        if (rd == 0) {
            return 0;
        } else {
            return self.regs[rd];
        }
    }

    fn read_csr(self: Self, csr: u12) CPUError!WORD {
        switch (csr) {
            CSR_MHARTID => {
                return self.hardware_thread_id;
            },
            CSR_MTVEC => {
                return self.trap_handler_base_address;
            },
            CSR_MNSTATUS => {
                log.warn("TODO: implement read CSR_MNSTATUS", .{});
                return 0;
            },
            CSR_SATP => {
                log.warn("TODO: implement read CSR_SATP", .{});
                return 0;
            },
            CSR_PMPADDR0 => {
                log.warn("TODO: implement read CSR_PMPADDR0", .{});
                return 0;
            },
            CSR_PMPCFG0 => {
                log.warn("TODO: implement read CSR_PMPCFG0", .{});
                return 0;
            },
            CSR_MIE => {
                log.warn("TODO: implement read MIE", .{});
                return 0;
            },
            CSR_MEDELEG => {
                log.warn("TODO: implement read CSR_MEDELEG", .{});
                return 0;
            },
            CSR_MIDELEG => {
                log.warn("TODO: implement read CSR_MIDELEG", .{});
                return 0;
            },
            CSR_MSTATUS => {
                log.warn("TODO: implement read CSR_MSTATUS", .{});
                return 0;
            },
            CSR_MEPC => {
                log.warn("TODO: implement read CSR_MEPC", .{});
                return 0;
            },
            else => {
                return CPUError.InvalidCSR;
            },
        }
    }

    fn write_csr(self: *Self, csr: u12, val: u32) CPUError!void {
        switch (csr) {
            CSR_MHARTID => {
                return CPUError.ReadOnlyCSR;
            },
            CSR_MTVEC => {
                self.trap_handler_base_address = val;
            },
            CSR_MNSTATUS => {
                log.warn("TODO: implement write CSR_MNSTATUS", .{});
            },
            CSR_SATP => {
                log.warn("TODO: implement write CSR_SATP", .{});
            },
            CSR_PMPADDR0 => {
                log.warn("TODO: implement write CSR_PMPADDR0", .{});
            },
            CSR_PMPCFG0 => {
                log.warn("TODO: implement write CSR_PMPCFG0", .{});
            },
            CSR_MIE => {
                log.warn("TODO: implement write CSR_MIE", .{});
            },
            CSR_MEDELEG => {
                log.warn("TODO: implement write CSR_MEDELEG", .{});
            },
            CSR_MIDELEG => {
                log.warn("TODO: implement write CSR_MIDELEG", .{});
            },
            CSR_MSTATUS => {
                log.warn("TODO: implement write CSR_MSTATUS", .{});
            },
            CSR_MEPC => {
                log.warn("TODO: implement write CSR_MEPC", .{});
            },
            else => {
                return CPUError.InvalidCSR;
            },
        }
    }
};

// https://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
fn sign_ext(val: WORD, bit_width: WORD_BIT_WIDTH_PLUS_ONE) WORD {
    const m: WORD = std.math.shl(WORD, 1, bit_width - 1);
    const r = val & @subWithOverflow(std.math.shl(WORD, 1, bit_width), 1)[0];
    return @subWithOverflow((r ^ m), m)[0];
}

test "sign extension" {
    try std.testing.expectEqual(0xFFFF_FFFF, sign_ext(0b11, 2));
}

test "risc-v tests" {
    // zig fmt: off
    const test_files = [_][]const u8{
        "rv32ui-p-add.bin",
        "rv32ui-p-addi.bin",
        "rv32ui-p-and.bin",
        "rv32ui-p-andi.bin",
        "rv32ui-p-auipc.bin",
        "rv32ui-p-beq.bin",
        "rv32ui-p-bge.bin",
        "rv32ui-p-bgeu.bin",
        "rv32ui-p-blt.bin",
        "rv32ui-p-bltu.bin",
        "rv32ui-p-bne.bin",
        "rv32ui-p-fence_i.bin",
        "rv32ui-p-jal.bin",
        "rv32ui-p-jalr.bin",
        "rv32ui-p-lb.bin",
        "rv32ui-p-lbu.bin",
        "rv32ui-p-lh.bin",
        "rv32ui-p-lhu.bin",
        "rv32ui-p-lui.bin",
        "rv32ui-p-lw.bin",
        "rv32ui-p-ma_data.bin",
        "rv32ui-p-or.bin",
        "rv32ui-p-ori.bin",
        "rv32ui-p-sb.bin",
        "rv32ui-p-sh.bin",
        "rv32ui-p-simple.bin",
        "rv32ui-p-sll.bin",
        "rv32ui-p-slli.bin",
        "rv32ui-p-slt.bin",
        "rv32ui-p-slti.bin",
        "rv32ui-p-sltiu.bin",
        "rv32ui-p-sltu.bin",
        "rv32ui-p-sra.bin",
        "rv32ui-p-srai.bin",
        "rv32ui-p-srl.bin",
        "rv32ui-p-srli.bin",
        "rv32ui-p-sub.bin",
        "rv32ui-p-sw.bin",
        "rv32ui-p-xor.bin",
        "rv32ui-p-xori.bin",
    };
    // zig fmt: on
    var test_file_buffer = [_]u8{0} ** 10000;
    const test_dir = try std.fs.cwd().openDir("./riscv-tests/isa", .{});
    for (test_files) |test_file| {
        var f = try test_dir.openFile(test_file, .{});
        const read_size = try f.readAll(&test_file_buffer);
        std.debug.print("testing {s} (size={d})\n", .{ test_file, read_size });

        var c = CPU.init();
        try c.load_memory(test_file_buffer[0..read_size], 0);
        while (true) {
            c.tick_cycle() catch |err| switch (err) {
                CPUError.EcallInvoked => {
                    break;
                },
                else => {
                    return CPUError.IllegalInstruction;
                },
            };
        }

        try std.testing.expectEqual(0, c.read_reg(10));
    }
}
