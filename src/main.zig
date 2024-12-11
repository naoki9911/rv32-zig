const std = @import("std");
const cpu = @import("cpu.zig");

pub fn main() !void {
    var test_file_buffer = [_]u8{0} ** 10000;
    var f = try std.fs.cwd().openFile("./riscv-tests/isa/rv32ui-p-add.bin", .{});
    const read_size = try f.readAll(&test_file_buffer);
    std.debug.print("test file size = {d}\n", .{read_size});

    var c = cpu.CPU.init();
    try c.load_memory(test_file_buffer[0..read_size], 0);
    try c.tick_cycle();

    std.debug.print("OK!\n", .{});
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
