const std = @import("std");

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
                val |= @as(WORD, data[offset + word_i]) << @intCast((8 * word_i));
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
        std.debug.print("MEM[0x{x}] INST=0x{x}\n", .{ self.pc, inst });

        return CPUError.IllegalInstruction;
    }
};
