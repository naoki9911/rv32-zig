const std = @import("std");
const cpu = @import("cpu.zig");

pub fn main() !void {
    var test_file_buffer = [_]u8{0} ** 10000;
    var f = try std.fs.cwd().openFile("./riscv-tests/isa/rv32si-p-ma_fetch.bin", .{});
    const read_size = try f.readAll(&test_file_buffer);
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

    const res_val = c.read_reg(3);
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
