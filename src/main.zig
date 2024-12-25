const std = @import("std");
const cpu = @import("cpu.zig");
const argsParser = @import("args");
const console = @import("console.zig");

pub const std_options = .{
    // Set this to .info, .debug, .warn, or .err.
    .log_level = .info,
};

pub fn main() !void {
    var allocator = std.heap.GeneralPurposeAllocator(.{}){};
    const options = try argsParser.parseForCurrentProcess(struct {
        // This declares long options for double hyphen
        testing: bool = false,
        mode: enum { cpu, console } = .cpu,
    }, allocator.allocator(), .print);
    defer options.deinit();

    std.log.info("mode={}", .{options.options.mode});

    switch (options.options.mode) {
        .cpu => {
            //var f = try std.fs.cwd().openFile("./riscv-tests/isa/rv32ui-v-add.bin", .{});
            var f = try std.fs.cwd().openFile("./xv6-riscv/kernel/kernel.bin", .{});
            const fstat = try f.stat();
            var test_file_buffer = try allocator.allocator().alloc(u8, fstat.size);
            defer allocator.allocator().free(test_file_buffer);
            const read_size = try f.readAll(test_file_buffer);
            std.debug.print("test file size = {d}\n", .{read_size});

            var c = try cpu.CPU.init(allocator.allocator());
            if (options.options.testing) {
                c.exit_on_ecall = true;
            }
            try c.load_memory(test_file_buffer[0..read_size], 0);
            try c.con.startConsoleServer();

            _ = try std.Thread.spawn(.{}, handleConsole, .{&c});
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
        },
        .console => {
            var client = try console.Console.init();
            try client.clientStart();
        },
    }
}

fn handleConsole(core: *cpu.CPU) !void {
    var before: u8 = 0;
    while (true) {
        const c = try core.con.readByte();
        // ctrl-a + x is exit key
        if (before == 0x1 and c == 'x') {
            break;
        }
        core.uart.putc(c);
        before = c;
    }
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
