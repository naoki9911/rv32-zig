const std = @import("std");
const cpu = @import("cpu.zig");

pub const std_options = .{
    // Set this to .info, .debug, .warn, or .err.
    .log_level = .warn,
};

pub fn main() !void {
    var allocator = std.heap.GeneralPurposeAllocator(.{}){};
    //var f = try std.fs.cwd().openFile("./riscv-tests/isa/rv32ui-v-add.bin", .{});
    var f = try std.fs.cwd().openFile("./xv6-riscv/kernel/kernel.bin", .{});
    const fstat = try f.stat();
    var test_file_buffer = try allocator.allocator().alloc(u8, fstat.size);
    defer allocator.allocator().free(test_file_buffer);
    const read_size = try f.readAll(test_file_buffer);
    std.debug.print("test file size = {d}\n", .{read_size});

    var buffer: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);

    var c = cpu.CPU.init(fba.allocator());
    c.exit_on_ecall = true;
    try c.load_memory(test_file_buffer[0..read_size], 0);
    while (true) {
        c.tick_cycle() catch |err| switch (err) {
            cpu.CPUError.EcallInvoked => {
                break;
            },
            else => {
                return cpu.CPUError.IllegalInstruction;
            },
        };
    }

    const res_val = c.read_reg(10);
    if (res_val == 1) {
        std.debug.print("OK!\n", .{});
    } else {
        std.debug.print("FAIL val={}\n", .{res_val});
    }
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
