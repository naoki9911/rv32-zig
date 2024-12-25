const std = @import("std");
const os = std.os;
const linux = os.linux;

pub const Console = struct {
    server: std.net.Server = undefined,
    client: std.net.Server.Connection = undefined,
    client_closed: bool = true,
    log_suppressed: bool = false,
    termios: Termios = .{},

    const Self = @This();
    const SockPath: []const u8 = "/tmp/rv32-zig-console.sock";

    pub fn init() !Self {
        return .{};
    }

    pub fn startConsoleServer(self: *Self) !void {
        std.fs.deleteFileAbsolute(SockPath) catch |err| {
            std.log.warn("failed to unlink err={}", .{err});
        };
        const addr = try std.net.Address.initUnix(SockPath);
        self.server = try addr.listen(.{});

        std.log.info("waiting console client connection...", .{});
        try self.accept();
        self.client_closed = false;

        const thread_config = std.Thread.SpawnConfig{};
        _ = try std.Thread.spawn(thread_config, serverThread, .{self});
    }

    fn serverThread(self: *Self) !void {
        while (true) {
            if (!self.client_closed) {
                std.time.sleep(1 * std.time.ns_per_s);
                continue;
            }

            // wait for client conneciton;
            try self.accept();
            std.log.info("accepted client connection!", .{});
            self.client_closed = false;
        }
    }

    pub fn write(self: *Self, value: []u8) !void {
        _ = self.client.stream.write(value) catch |err| {
            if (!self.log_suppressed) {
                std.log.warn("console output is unavailable err={}", .{err});
                self.log_suppressed = true;
            }
        };
    }

    pub fn readByte(self: *Self) !u8 {
        while (true) {
            const res = self.client.stream.reader().readByte() catch |err| switch (err) {
                error.WouldBlock => continue,
                error.EndOfStream => {
                    // wait for console client connection
                    std.time.sleep(1 * std.time.ns_per_s);
                    self.client_closed = true;
                    continue;
                },
                else => return err,
            };
            return res;
        }
    }

    pub fn clientStart(self: *Self) !void {
        self.termios = try Termios.get();
        defer {
            self.termios.set() catch |err| {
                std.log.warn("failed to restore termios err={}", .{err});
            };
            std.log.info("restored termios", .{});
        }
        var s = try self.connect();
        std.log.info("connected to rv32-zig!", .{});

        try Termios.setRawMode();
        std.log.info("configured terminal to raw mode", .{});

        const thread_config = std.Thread.SpawnConfig{};
        const stdout_thread = try std.Thread.spawn(thread_config, handleStdout, .{&s});
        _ = stdout_thread;

        const stdin = std.io.getStdIn().reader();
        var before: u8 = 0;
        while (true) {
            const c = try stdin.readByte();
            // ctrl-a + x is exit key
            if (before == 0x1 and c == 'x') {
                break;
            }
            before = c;
            s.writer().writeByte(c) catch |err| {
                std.log.err("disconnected err={}", .{err});
                break;
            };
        }
    }

    fn accept(self: *Self) !void {
        std.log.info("waiting for connection", .{});
        self.client = try self.server.accept();

        std.log.info("accepted!", .{});
        // configure timeout
        self.log_suppressed = false;
        try setReadTimeout(self.client.stream.handle, 1000);
    }

    fn handleStdout(stream: *std.net.Stream) !void {
        var buf = [_]u8{0} ** 100;
        while (true) {
            const readSize = stream.read(&buf) catch |err| {
                std.log.err("disconnected err={}", .{err});
                break;
            };
            _ = try std.io.getStdOut().write(buf[0..readSize]);
        }
    }

    fn connect(self: *Self) !std.net.Stream {
        _ = self;
        return try std.net.connectUnixSocket(SockPath);
    }
};

fn setReadTimeout(sock: std.posix.socket_t, milliseconds: usize) !void {
    const timeout = std.posix.timeval{
        .tv_sec = @intCast(milliseconds / std.time.ms_per_s),
        .tv_usec = @intCast((milliseconds % std.time.ms_per_s) * std.time.us_per_ms),
    };

    try std.posix.setsockopt(sock, std.posix.SOL.SOCKET, std.posix.SO.RCVTIMEO, std.mem.asBytes(&timeout));
}

// linux/include/uapi/asm-generic/termbits.h
pub const Termios = extern struct {
    iflag: u32 = 0,
    oflag: u32 = 0,
    cflag: u32 = 0,
    lflag: u32 = 0,
    line: u8 = 0,
    cc: [19]u8 = [_]u8{0} ** 19,
    ispeed: u32 = 0,
    ospeed: u32 = 0,

    const TCGETS = 0x5401;
    const TCSETS = 0x5402;

    // c_iflag bits
    const IGNBRK: u32 = 0x001;
    const BRKINT: u32 = 0x002;
    const IGNPAR: u32 = 0x004;
    const PARMRK: u32 = 0x008;
    const INPCK: u32 = 0x010;
    const ISTRIP: u32 = 0x020;
    const INLCR: u32 = 0x040;
    const IGNCR: u32 = 0x080;
    const ICRNL: u32 = 0x100;
    const IXON: u32 = 0x0400;
    const IXANY: u32 = 0x800;

    // c_oflag bits
    const OPOST: u32 = 0x01;

    // c_cflag bits
    const CSIZE: u32 = 0x00000030;
    const PARENB: u32 = 0x00000100;
    const CS8: u32 = 0x00000030;

    // c_lflag bits
    const ISIG: u32 = 0x00001;
    const ICANON: u32 = 0x00002;
    const ECHO: u32 = 0x00008;
    const ECHONL: u32 = 0x00040;
    const IEXTEN: u32 = 0x08000;

    // c_cc characters
    const VTIME: u32 = 5;
    const VMIN: u32 = 6;

    const Self = @This();
    const Error = error{
        FailedToGetTermios,
        FailedToSetTermios,
    };

    pub fn get() !Self {
        var t = Self{};
        const ret = linux.ioctl(0, TCGETS, @intFromPtr(&t));
        if (ret != 0) {
            return Error.FailedToGetTermios;
        }

        return t;
    }

    pub fn set(self: *const Self) !void {
        const ret = linux.ioctl(0, TCSETS, @intFromPtr(self));
        if (ret != 0) {
            return Error.FailedToSetTermios;
        }
    }

    pub fn setRawMode() !void {
        var t = try Self.get();

        // modified termios(3) Raw mode
        t.iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        t.cflag &= ~(CSIZE | PARENB);
        t.lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        t.cc[VMIN] = 1;
        t.cc[VTIME] = 0;

        try t.set();
    }
};
