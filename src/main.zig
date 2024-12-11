const std = @import("std");
const cpu = @import("cpu.zig");

pub fn main() !void {
    var c = cpu.CPU.init();
    try c.load_memory(&[_]u8{ 0, 1, 2 }, 0);
    std.debug.print("OK!\n", .{});
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
