const std = @import("std");

const WORD = u32;
const MEMORY_SIZE = 0x10000; // 64KiB
const MEMORY_BASE_ADDR = 0x80000000; // riscv-tests

const CPUError = error{TooLargeMemoryData};

pub const CPU = struct {
    regs: [32]WORD,
    mem: [MEMORY_SIZE]u8,

    const Self = @This();

    pub fn init() Self {
        return .{
            .regs = [1]WORD{0} ** 32,
            .mem = [1]u8{0} ** MEMORY_SIZE,
        };
    }

    pub fn load_memory(self: *Self, data: []const u8, offset: WORD) CPUError!void {
        if (data.len > MEMORY_SIZE + offset) {
            return CPUError.TooLargeMemoryData;
        }
        @memcpy(self.mem[offset .. offset + data.len], data);
    }
};
