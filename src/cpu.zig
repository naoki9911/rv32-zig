const std = @import("std");
const log = std.log;

const WORD = u32;
const MEMORY_SIZE = 0x10000; // 64KiB
const MEMORY_BASE_ADDR = 0x80000000; // riscv-tests

const CPUError = error{
    TooLargeMemoryData,
    OutOfMemoryArea,
    IllegalInstruction,
};

pub const CPU = struct {
    pc: WORD,
    regs: [32]WORD,
    mem: [MEMORY_SIZE >> 2]WORD,

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

        const inst_addr = self.pc - MEMORY_BASE_ADDR;

        // instructions are stored in 4 bytes aligned
        const inst = self.mem[inst_addr >> 2];
        log.debug("MEM[0x{x}] INST=0b{b} (0x{x})", .{ self.pc, inst, inst });

        if (inst == 0) {
            return CPUError.IllegalInstruction;
        }
        const rd = (inst >> 7) & 0x1F; // rd is inst[11:7]
        switch (inst & 0x7F) {
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
                self.regs[rd] = self.pc;
                self.pc += imm; // adding any values including negative one
                return;
            },
            else => return CPUError.IllegalInstruction,
        }

        self.pc += 1;
    }

    fn read_reg(self: *Self, rd: u5) WORD {
        if (rd == 0) {
            return 0;
        } else {
            return self.regs[rd];
        }
    }
};
