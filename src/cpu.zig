const std = @import("std");
const log = std.log;

const WORD = u32;
const WORD_BIT_WIDTH_PLUS_ONE = u6;
const MEMORY_SIZE = 0x100000; // 128KiB
const MEMORY_BASE_ADDR = 0x80000000; // riscv-tests

const CSR = struct {
    const PrivilegeLevels = enum(u2) {
        User = 0b00,
        Supervisor = 0b01,
        Reserved = 0b10,
        Machine = 0b11,
    };

    // zig fmt: off
    const Registers = enum(u12) {
        // Machine Information Registers
        mvendorid  = 0xF11, // MRO, Vendor ID
        marchid    = 0xF12, // MRO, Architectuere ID
        mimpid     = 0xF13, // MRO, Implementation ID
        mhartid    = 0xF14, // MRO, Hardware thread ID
        mconfigptr = 0xF15, // MRO, Pointer to configuration data structure

        // Machine Trap Setup
        mstatus    = 0x300, // MRW, Machine status register.
        misa       = 0x301, // MRW, ISA and extensions
        medeleg    = 0x302, // MRW, Machine exception delegation register.
        mideleg    = 0x303, // MRW, Machine interrupt delegation register.
        mie        = 0x304, // MRW, Machine interrupt-enable register.
        mtvec      = 0x305, // MRW, Machine trap-handler base address.
        mcounteren = 0x306, // MRW, Machine counter enable.
        mstatush   = 0x310, // MRW, Additional machine status register, RV32 only.
        medelegh   = 0x312, // MRW, Upper 32 bits of medeleg, RV32 only.

        // Machine Trap Handling
        mscratch = 0x340, // MRW, Scratch register for machine trap handlers.
        mepc     = 0x341, // MRW, Machine exception program counter.
        mcause   = 0x342, // MRW, Machine trap cause.
        mtval    = 0x343, // MRW, Machine bad address or instruction.
        mip      = 0x344, // MRW, Machine interrupt pending.
        mtinst   = 0x34A, // MRW, Machine trap instruction (transformed).
        mtval2   = 0x34B, // MRW, Machine bad guest physical address.

        // Machine Memory Protection
        pmpcfg0   = 0x3A0, // MRW, Physical memory protection configuration.
        pmpcfg1   = 0x3A1, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg2   = 0x3A2, // MRW, Physical memory protection configuration.
        pmpcfg3   = 0x3A3, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg4   = 0x3A4, // MRW, Physical memory protection configuration.
        pmpcfg5   = 0x3A5, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg6   = 0x3A6, // MRW, Physical memory protection configuration.
        pmpcfg7   = 0x3A7, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg8   = 0x3A8, // MRW, Physical memory protection configuration.
        pmpcfg9   = 0x3A9, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg10  = 0x3AA, // MRW, Physical memory protection configuration.
        pmpcfg11  = 0x3AB, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg12  = 0x3AC, // MRW, Physical memory protection configuration.
        pmpcfg13  = 0x3AD, // MRW, Physical memory protection configuration. RV32 only
        pmpcfg14  = 0x3AE, // MRW, Physical memory protection configuration.
        pmpcfg15  = 0x3AF, // MRW, Physical memory protection configuration. RV32 only
        pmpaddr0  = 0x3B0, // MRW, Physical memory protection address register
        pmpaddr1  = 0x3B1, // MRW, Physical memory protection address register
        pmpaddr2  = 0x3B2, // MRW, Physical memory protection address register
        pmpaddr3  = 0x3B3, // MRW, Physical memory protection address register
        pmpaddr4  = 0x3B4, // MRW, Physical memory protection address register
        pmpaddr5  = 0x3B5, // MRW, Physical memory protection address register
        pmpaddr6  = 0x3B6, // MRW, Physical memory protection address register
        pmpaddr7  = 0x3B7, // MRW, Physical memory protection address register
        pmpaddr8  = 0x3B8, // MRW, Physical memory protection address register
        pmpaddr9  = 0x3B9, // MRW, Physical memory protection address register
        pmpaddr10 = 0x3BA, // MRW, Physical memory protection address register
        pmpaddr11 = 0x3BB, // MRW, Physical memory protection address register
        pmpaddr12 = 0x3BC, // MRW, Physical memory protection address register
        pmpaddr13 = 0x3BD, // MRW, Physical memory protection address register
        pmpaddr14 = 0x3BE, // MRW, Physical memory protection address register
        pmpaddr15 = 0x3BF, // MRW, Physical memory protection address register
        pmpaddr16 = 0x3C0, // MRW, Physical memory protection address register
        pmpaddr17 = 0x3C1, // MRW, Physical memory protection address register
        pmpaddr18 = 0x3C2, // MRW, Physical memory protection address register
        pmpaddr19 = 0x3C3, // MRW, Physical memory protection address register
        pmpaddr20 = 0x3C4, // MRW, Physical memory protection address register
        pmpaddr21 = 0x3C5, // MRW, Physical memory protection address register
        pmpaddr22 = 0x3C6, // MRW, Physical memory protection address register
        pmpaddr23 = 0x3C7, // MRW, Physical memory protection address register
        pmpaddr24 = 0x3C8, // MRW, Physical memory protection address register
        pmpaddr25 = 0x3C9, // MRW, Physical memory protection address register
        pmpaddr26 = 0x3CA, // MRW, Physical memory protection address register
        pmpaddr27 = 0x3CB, // MRW, Physical memory protection address register
        pmpaddr28 = 0x3CC, // MRW, Physical memory protection address register
        pmpaddr29 = 0x3CD, // MRW, Physical memory protection address register
        pmpaddr30 = 0x3CE, // MRW, Physical memory protection address register
        pmpaddr31 = 0x3CF, // MRW, Physical memory protection address register
        pmpaddr32 = 0x3D0, // MRW, Physical memory protection address register
        pmpaddr33 = 0x3D1, // MRW, Physical memory protection address register
        pmpaddr34 = 0x3D2, // MRW, Physical memory protection address register
        pmpaddr35 = 0x3D3, // MRW, Physical memory protection address register
        pmpaddr36 = 0x3D4, // MRW, Physical memory protection address register
        pmpaddr37 = 0x3D5, // MRW, Physical memory protection address register
        pmpaddr38 = 0x3D6, // MRW, Physical memory protection address register
        pmpaddr39 = 0x3D7, // MRW, Physical memory protection address register
        pmpaddr40 = 0x3D8, // MRW, Physical memory protection address register
        pmpaddr41 = 0x3D9, // MRW, Physical memory protection address register
        pmpaddr42 = 0x3DA, // MRW, Physical memory protection address register
        pmpaddr43 = 0x3DB, // MRW, Physical memory protection address register
        pmpaddr44 = 0x3DC, // MRW, Physical memory protection address register
        pmpaddr45 = 0x3DD, // MRW, Physical memory protection address register
        pmpaddr46 = 0x3DE, // MRW, Physical memory protection address register
        pmpaddr47 = 0x3DF, // MRW, Physical memory protection address register
        pmpaddr48 = 0x3E0, // MRW, Physical memory protection address register
        pmpaddr49 = 0x3E1, // MRW, Physical memory protection address register
        pmpaddr50 = 0x3E2, // MRW, Physical memory protection address register
        pmpaddr51 = 0x3E3, // MRW, Physical memory protection address register
        pmpaddr52 = 0x3E4, // MRW, Physical memory protection address register
        pmpaddr53 = 0x3E5, // MRW, Physical memory protection address register
        pmpaddr54 = 0x3E6, // MRW, Physical memory protection address register
        pmpaddr55 = 0x3E7, // MRW, Physical memory protection address register
        pmpaddr56 = 0x3E8, // MRW, Physical memory protection address register
        pmpaddr57 = 0x3E9, // MRW, Physical memory protection address register
        pmpaddr58 = 0x3EA, // MRW, Physical memory protection address register
        pmpaddr59 = 0x3EB, // MRW, Physical memory protection address register
        pmpaddr60 = 0x3EC, // MRW, Physical memory protection address register
        pmpaddr61 = 0x3ED, // MRW, Physical memory protection address register
        pmpaddr62 = 0x3EE, // MRW, Physical memory protection address register
        pmpaddr63 = 0x3EF, // MRW, Physical memory protection address register


        // "Smrnmi" Extension for Resumable Non-Maskable Interrupts,
        // Machine Non-Maskable Interrupt Handling
        // TODO: support this
        mnscratch  = 0x740, // MRW, Resumable NMI scratch register.
        mnepc      = 0x741, // MRW, Resumable NMI program counter.
        mncause    = 0x742, // MRW, Resumable NMI cause.
        mnstatus   = 0x744, // MRW, Resumable NMI status.

        // Machine Trap Setup
        // Debug related CSRs
        // https://github.com/riscv/riscv-debug-spec/blob/release/riscv-debug-release.pdf
        tselect  = 0x7A0, // Trigger select
        tdata1   = 0x7A1, // Trigger select
        tdata2   = 0x7A2, // Trigger select
        tdata3   = 0x7A3, // Trigger select
        tcontrol = 0x7A5, // Trigger control


        // Supervisor Trap Setup
        sstatus    = 0x100, // SRW, Supervisor status register.
        sie        = 0x104, // SRW, Supervisor interrupt-enable register.
        stvec      = 0x105, // SRW, Supervisor trap handler base address.
        scounteren = 0x106, // SRW, Supervisor counter enable.
    
        // Supervisor Trap Handling
        sscratch  = 0x140,  // SRW, Scratch register for supervisor trap handlers.
        sepc      = 0x141,  // SRW, Supervisor exception program counter.
        scause    = 0x142,  // SRW, Supervisor trap cause.
        stval     = 0x143,  // SRW, Supervisor bad address or instruction.
        sip       = 0x144,  // SRW, Supervisor interrupt pending.
        scountovf = 0xDA0,  // SRO, Supervisor count overflow.

        // Supervisor Protection and Translation
        satp = 0x180, // SRW, Supervisor address translation and protection.
        _,
    };
    // zig fmt: on

    const RegMstatus = packed struct(u32) {
        wpri1: u1 = 0,
        sie: u1 = 0,
        wpri2: u1 = 0,
        mie: u1 = 0,
        wpri3: u1 = 0,
        spie: u1 = 0,
        ube: u1 = 0,
        mpie: u1 = 0,
        spp: u1 = 0,
        vs: u2 = 0,
        mpp: u2 = 0,
        fs: u2 = 0,
        xs: u2 = 0,
        mprv: u1 = 0,
        sum: u1 = 0,
        mxr: u1 = 0,
        tvm: u1 = 0,
        tw: u1 = 0,
        tsr: u1 = 0,
        wpri4: u8 = 0,
        sd: u1 = 0,
    };

    const RegMstatush = packed struct(u32) {
        wpri1: u4 = 0,
        sbe: u1 = 0,
        mbe: u1 = 0,
        wpri2: u26 = 0,
    };

    const RegTvec = packed struct(u32) {
        mode: u2 = 0,
        base: u30 = 0,
    };
    const RegPmpConfig = packed struct(u8) {
        r: u1 = 0,
        w: u1 = 0,
        x: u1 = 0,
        a: u2 = 0,
        reserved_zero: u2 = 0,
        l: u1 = 0,
    };

    // RV32IMA-MSU
    const RegMisa = packed struct(u32) {
        a: u1 = 1,
        b: u1 = 0,
        c: u1 = 0,
        d: u1 = 0,
        e: u1 = 0,
        f: u1 = 0,
        g: u1 = 0,
        h: u1 = 0,
        i: u1 = 1,
        j: u1 = 0,
        k: u1 = 0,
        l: u1 = 0,
        m: u1 = 1,
        n: u1 = 0,
        o: u1 = 0,
        p: u1 = 0,
        q: u1 = 0,
        r: u1 = 0,
        s: u1 = 1,
        t: u1 = 0,
        u: u1 = 1,
        v: u1 = 0,
        w: u1 = 0,
        x: u1 = 0,
        y: u1 = 0,
        z: u1 = 0,
        zero: u4 = 0,
        mxl: u2 = 1,
    };

    const RegMcause = packed struct(u32) {
        exception_code: u31,
        interrupt: u1,
    };

    const RegMip = packed struct(u32) {
        zero1: u1 = 0,
        ssip: u1 = 0,
        zero2: u1 = 0,
        msip: u1 = 0,
        zero3: u1 = 0,
        stip: u1 = 0,
        zero4: u1 = 0,
        mtip: u1 = 0,
        zero5: u1 = 0,
        seip: u1 = 0,
        zero6: u1 = 0,
        meip: u1 = 0,
        zero7: u1 = 0,
        lcofip: u1 = 0,
        zero8: u2 = 0,
        zero9: u16 = 0,
    };

    const RegMie = packed struct(u32) {
        zero1: u1 = 0,
        ssie: u1 = 0,
        zero2: u1 = 0,
        msie: u1 = 0,
        zero3: u1 = 0,
        stie: u1 = 0,
        zero4: u1 = 0,
        mtie: u1 = 0,
        zero5: u1 = 0,
        seie: u1 = 0,
        zero6: u1 = 0,
        meie: u1 = 0,
        zero7: u1 = 0,
        lcofie: u1 = 0,
        zero8: u2 = 0,
        zero9: u16 = 0,
    };

    const RegSstatus = packed struct(u32) {
        wpri1: u1 = 0,
        sie: u1 = 0,
        wpri2: u3 = 0,
        spie: u1 = 0,
        ube: u1 = 0,
        wpri3: u1 = 0,
        spp: u1 = 0,
        vs: u2 = 0,
        wpri4: u2 = 0,
        fs: u2 = 0,
        xs: u2 = 0,
        wpri5: u1 = 0,
        sum: u1 = 0,
        mxr: u1 = 0,
        wpri6: u11 = 0,
        sd: u1 = 0,
    };

    const RegSatp = packed struct(u32) {
        ppn: u22 = 0,
        asid: u9 = 0,
        mode: u1 = 0,
    };

    const Self = @This();

    // TODO: when the core initialized, entering machine mode is fine?
    current_level: PrivilegeLevels = .Machine,
    hardware_thread_id: WORD = 0,
    reg_mtvec: RegTvec = .{},
    reg_mstatus: RegMstatus = .{},
    reg_mstatush: RegMstatush = .{},
    reg_pmpcfg: [64]RegPmpConfig = [1]RegPmpConfig{.{}} ** 64,
    reg_pmpaddr: [64]WORD = [1]WORD{0} ** 64,
    reg_mip: RegMip = .{},
    reg_mie: RegMie = .{},
    reg_mepc: WORD = 0,
    reg_mscratch: WORD = 0,
    reg_mcause: RegMcause = .{ .exception_code = 0, .interrupt = 0 },
    reg_mtval: WORD = 0,
    reg_medeleg: u64 = 0,
    reg_mideleg: RegMip = .{},

    reg_satp: RegSatp = .{},
    reg_stvec: RegTvec = .{},
    reg_sepc: WORD = 0,
    reg_sscratch: WORD = 0,
    reg_scause: RegMcause = .{ .exception_code = 0, .interrupt = 0 },
    reg_stval: WORD = 0,

    sv32_enabled: bool = false,

    // Machine-mode standard read-write CSRs 0x7A0-0x7BF are reserved for use by the debug system. Of
    // these CSRs, 0x7A0-0x7AF are accessible to machine mode, whereas 0x7B0-0x7BF are only visible to
    // debug mode. Implementations should raise illegal-instruction exceptions on machine-mode access to
    // the latter set of registers.
    debug_mode: bool = false,

    fn init() Self {
        return .{};
    }

    fn read(self: Self, csr: u12) CPUError!WORD {
        const level = @as(u2, @intCast((csr >> 8) & 0x3));
        if (level > @as(u2, @intFromEnum(self.current_level))) {
            log.warn("[CSR.READ] insufficient privilege require={} current={}", .{ @as(Self.PrivilegeLevels, @enumFromInt(level)), self.current_level });
            return CPUError.IllegalInstruction;
        }

        const c: Registers = @enumFromInt(csr);
        switch (c) {
            .mvendorid, .marchid, .mimpid, .mconfigptr => {
                return 0;
            },
            .mhartid => {
                return self.hardware_thread_id;
            },
            .mstatus => {
                log.debug("[CSR.READ] mstatus read {}", .{self.reg_mstatus});
                return @bitCast(self.reg_mstatus);
            },
            .misa => {
                log.debug("[CSR.READ] misa read {}", .{RegMisa{}});
                return @bitCast(RegMisa{});
            },
            .medeleg => {
                const res: u32 = @intCast(self.reg_medeleg & 0xFFFFFFFF);
                log.debug("[CSR.READ medeleg 0x{x}]", .{res});
                return res;
            },
            .medelegh => {
                const res: u32 = @intCast((self.reg_medeleg >> 32) & 0xFFFFFFFF);
                log.debug("[CSR.READ medelegh 0x{x}]", .{res});
                return res;
            },
            .mideleg => {
                log.debug("[CSR.READ mideleg {}]", .{self.reg_mideleg});
                return @bitCast(self.reg_mideleg);
            },
            .mie => {
                log.debug("[CSR.READ] mie read {}", .{self.reg_mie});
                return @bitCast(self.reg_mie);
            },
            .mip => {
                log.debug("[CSR.READ] mip write {}", .{self.reg_mip});
                return @bitCast(self.reg_mip);
            },
            .mtvec => {
                log.debug("[CSR.READ] mtvec read {}", .{self.reg_mtvec});
                return @bitCast(self.reg_mtvec);
            },
            .mscratch => {
                log.debug("[CSR.READ] mscrath read 0x{x}", .{self.reg_mscratch});
                return self.reg_mscratch;
            },
            .mepc => {
                log.debug("[CSR.READ] mepc read 0x{x}", .{self.reg_mepc});
                return self.reg_mepc;
            },
            .mtval => {
                log.debug("[CSR.READ] mtval read 0x{x}", .{self.reg_mtval});
                return self.reg_mtval;
            },
            .mcause => {
                log.debug("[CSR.READ] mcause read {}", .{self.reg_mcause});
                return @bitCast(self.reg_mcause);
            },
            // zig fmt: off
            .pmpcfg0,  .pmpcfg1,  .pmpcfg2,  .pmpcfg3,
            .pmpcfg4,  .pmpcfg5,  .pmpcfg6,  .pmpcfg7,
            .pmpcfg8,  .pmpcfg9,  .pmpcfg10, .pmpcfg11,
            .pmpcfg12, .pmpcfg13, .pmpcfg14, .pmpcfg15,
            // zig fmt: on
            => {
                const idx_base = (csr - @intFromEnum(Registers.pmpcfg0)) * 4;
                return std.math.shl(u32, @as(u8, @bitCast(self.reg_pmpcfg[idx_base + 3])), 24) |
                    std.math.shl(u32, @as(u8, @bitCast(self.reg_pmpcfg[idx_base + 2])), 16) |
                    std.math.shl(u32, @as(u8, @bitCast(self.reg_pmpcfg[idx_base + 1])), 8) |
                    @as(u8, @bitCast(self.reg_pmpcfg[idx_base]));
            },
            // zig fmt: off
            .pmpaddr0,  .pmpaddr1,  .pmpaddr2,  .pmpaddr3,  .pmpaddr4,  .pmpaddr5,  .pmpaddr6,  .pmpaddr7,
            .pmpaddr8,  .pmpaddr9,  .pmpaddr10, .pmpaddr11, .pmpaddr12, .pmpaddr13, .pmpaddr14, .pmpaddr15,
            .pmpaddr16, .pmpaddr17, .pmpaddr18, .pmpaddr19, .pmpaddr20, .pmpaddr21, .pmpaddr22, .pmpaddr23,
            .pmpaddr24, .pmpaddr25, .pmpaddr26, .pmpaddr27, .pmpaddr28, .pmpaddr29, .pmpaddr30, .pmpaddr31,
            .pmpaddr32, .pmpaddr33, .pmpaddr34, .pmpaddr35, .pmpaddr36, .pmpaddr37, .pmpaddr38, .pmpaddr39,
            .pmpaddr40, .pmpaddr41, .pmpaddr42, .pmpaddr43, .pmpaddr44, .pmpaddr45, .pmpaddr46, .pmpaddr47,
            .pmpaddr48, .pmpaddr49, .pmpaddr50, .pmpaddr51, .pmpaddr52, .pmpaddr53, .pmpaddr54, .pmpaddr55,
            .pmpaddr56, .pmpaddr57, .pmpaddr58, .pmpaddr59, .pmpaddr60, .pmpaddr61, .pmpaddr62, .pmpaddr63,
            // zig fmt: on
            => {
                const val = self.reg_pmpaddr[csr - @intFromEnum(Registers.pmpaddr0)];
                log.debug("[CSR.READ] {} read 0x{x}", .{ c, val });
                return val;
            },
            .mnscratch, .mnepc, .mncause, .mnstatus => {
                log.warn("Smrnmi extension is not supported", .{});
                return 0;
            },
            .sie => {
                // sie is a view of mie
                const sie = RegMie{
                    .ssie = self.reg_mie.ssie,
                    .stie = self.reg_mie.stie,
                    .seie = self.reg_mie.seie,
                    .lcofie = self.reg_mie.lcofie,
                };
                log.debug("[CSR.READ] sie read {}", .{sie});
                return @bitCast(sie);
            },
            .stvec => {
                log.debug("[CSR.READ] stvec read {}", .{self.reg_stvec});
                return @bitCast(self.reg_stvec);
            },
            .scause => {
                log.debug("[CSR.READ] scause read {}", .{self.reg_scause});
                return @bitCast(self.reg_scause);
            },
            .stval => {
                log.debug("[CSR.READ] stval read 0x{x}", .{self.reg_stval});
                return self.reg_stval;
            },
            .sepc => {
                log.debug("[CSR.READ] spec read 0x{x}", .{self.reg_sepc});
                return @bitCast(self.reg_sepc);
            },
            .sscratch => {
                log.debug("[CSR.READ] sscrach read 0x{x}", .{self.reg_sscratch});
                return self.reg_sscratch;
            },
            .sstatus => {
                const ss = RegSstatus{
                    .sie = self.reg_mstatus.sie,
                    .spie = self.reg_mstatus.spie,
                    .ube = self.reg_mstatus.ube,
                    .spp = self.reg_mstatus.spp,
                    .vs = self.reg_mstatus.vs,
                    .fs = self.reg_mstatus.fs,
                    .xs = self.reg_mstatus.xs,
                    .sum = self.reg_mstatus.sum,
                    .mxr = self.reg_mstatus.mxr,
                    .sd = self.reg_mstatus.sd,
                };
                log.debug("[CSR.READ] sstatus read {}", .{ss});
                return @bitCast(ss);
            },
            .sip => {
                const sip = RegMip{
                    .ssip = self.reg_mip.ssip,
                    .stip = self.reg_mip.stip,
                    .seip = self.reg_mip.seip,
                    .lcofip = self.reg_mip.lcofip,
                };
                log.debug("[CSR.READ] sip read {}", .{sip});
                return @bitCast(sip);
            },
            .satp => {
                // When TVM=1, attempts to read or write the satp CSR or execute an
                // SFENCE.VMA or SINVAL.VMA instruction while executing in S-mode will raise an illegal-instruction
                // exception. When TVM=0, these operations are permitted in S-mode. TVM is read-only 0 when Smode is not supported.
                if (self.reg_mstatus.tvm == 1 and self.current_level == .Supervisor) return CPUError.IllegalInstruction;
                log.debug("[CSR.READ] satp = {}", .{self.reg_satp});
                return @bitCast(self.reg_satp);
            },
            else => {
                log.warn("TODO: read from CSR {} (0x{x}) is not handled", .{ c, csr });
                return 0;
            },
        }
    }

    fn check_writable(csr: u12) CPUError!void {
        if (csr >> 10 == 0b11) {
            return CPUError.IllegalInstruction;
        }
    }

    fn write(self: *Self, csr: u12, val: u32) CPUError!void {
        const level = @as(u2, @intCast((csr >> 8) & 0x3));
        if (level > @as(u2, @intFromEnum(self.current_level))) {
            log.warn("[CSR.READ] insufficient privilege require={} current={}", .{ @as(Self.PrivilegeLevels, @enumFromInt(level)), self.current_level });
            return CPUError.IllegalInstruction;
        }

        const c: Registers = @enumFromInt(csr);
        if (csr >> 10 == 0b11) {
            // readonly csr registers
            log.debug("CSR {} is readonly", .{c});
            return CPUError.IllegalInstruction;
        }
        switch (c) {
            .mstatus => {
                self.reg_mstatus = @bitCast(val);

                // memory access for M, S, U mode is explicitly little-endian
                self.reg_mstatush.mbe = 0;
                self.reg_mstatush.sbe = 0;
                self.reg_mstatus.ube = 0;

                // F extension is not implemented
                self.reg_mstatus.fs = 0;
                // V extension is not implemented
                self.reg_mstatus.vs = 0;
                // F and V extensions are 0
                self.reg_mstatus.xs = 0;
                self.reg_mstatus.sd = 0;
                log.debug("[CSR.WRITE] mstatus write {}", .{self.reg_mstatus});
            },
            .misa => {
                log.debug("[CSR.WRITE] misa write {}", .{@as(RegMisa, @bitCast(val))});
            },
            .medeleg => {
                self.reg_medeleg = (self.reg_medeleg & 0xFFFFFFFF_00000000) | @as(u64, @intCast(val));
                log.debug("[CSR.WRITE medeleg 0x{x}]", .{self.reg_medeleg});
            },
            .medelegh => {
                self.reg_medeleg = (self.reg_medeleg & 0x00000000_FFFFFFFF) | (@as(u64, @intCast(val)) << 32);
                log.debug("[CSR.WRITE medeleg 0x{x}]", .{self.reg_medeleg});
            },
            .mideleg => {
                self.reg_mideleg = @bitCast(val);
                log.debug("[CSR.WRITE mideleg {}]", .{self.reg_mideleg});
            },
            .mie => {
                self.reg_mie = @bitCast(val & 0xAAA);
                log.debug("[CSR.WRITE] mie write {}", .{self.reg_mie});
            },
            .mip => {
                self.reg_mip = @bitCast(val & 0xAAA);
                log.debug("[CSR.WRITE] mip write {}", .{self.reg_mip});
            },
            .mtvec => {
                self.reg_mtvec = @bitCast(val);
                log.debug("[CSR.WRITE] mtvec write {}", .{self.reg_mtvec});
            },
            .mscratch => {
                self.reg_mscratch = val;
                log.debug("[CSR.WRITE] mscrath write 0x{x}", .{self.reg_mscratch});
            },
            .mcause => {
                self.reg_mcause = @bitCast(val);
                log.debug("[CSR.WRITE] mcause write {}", .{self.reg_mcause});
            },
            .mtval => {
                self.reg_mtval = val;
                log.debug("[CSR.WRITE] mtval write 0x{x}", .{self.reg_mtval});
            },
            .mepc => {
                self.reg_mepc = val;
                log.debug("[CSR.WRITE] mepc write 0x{x}", .{self.reg_mepc});
            },
            // zig fmt: off
            .pmpcfg0,  .pmpcfg1,  .pmpcfg2,  .pmpcfg3,
            .pmpcfg4,  .pmpcfg5,  .pmpcfg6,  .pmpcfg7,
            .pmpcfg8,  .pmpcfg9,  .pmpcfg10, .pmpcfg11,
            .pmpcfg12, .pmpcfg13, .pmpcfg14, .pmpcfg15,
            // zig fmt: on
            => {
                const idx_base = (csr - @intFromEnum(Registers.pmpcfg0)) * 4;

                // reserved_zero is masked.
                // R = 0 and W = 1 is reserved.
                const cfg0: RegPmpConfig = @bitCast(@as(u8, @intCast(val & 0x9F)));
                if (cfg0.r == 0 and cfg0.w == 1) return CPUError.InvalidCSRState;

                const cfg1: RegPmpConfig = @bitCast(@as(u8, @intCast((val >> 8) & 0x9F)));
                if (cfg1.r == 0 and cfg1.w == 1) return CPUError.InvalidCSRState;

                const cfg2: RegPmpConfig = @bitCast(@as(u8, @intCast((val >> 16) & 0x9F)));
                if (cfg2.r == 0 and cfg2.w == 1) return CPUError.InvalidCSRState;

                const cfg3: RegPmpConfig = @bitCast(@as(u8, @intCast((val >> 24) & 0x9F)));
                if (cfg3.r == 0 and cfg3.w == 1) return CPUError.InvalidCSRState;

                log.debug("[CSR.WRITE] {} write cfg0={} cfg1={} cfg2={} cfg3={}", .{ c, cfg0, cfg1, cfg2, cfg3 });

                // The L bit indicates that the PMP entry is locked, i.e.,
                // writes to the configuration register and associated address registers are ignored
                // Additionally, if PMP entry i is locked and pmpicfg.A is set to TOR, writes to pmpaddri-1 are ignored.
                if (self.reg_pmpcfg[idx_base].l == 0) {
                    self.reg_pmpcfg[idx_base] = cfg0;
                }
                if (self.reg_pmpcfg[idx_base + 1].l == 0) {
                    self.reg_pmpcfg[idx_base + 1] = cfg1;
                }
                if (self.reg_pmpcfg[idx_base + 2].l == 0) {
                    self.reg_pmpcfg[idx_base + 2] = cfg2;
                }
                if (self.reg_pmpcfg[idx_base + 3].l == 0) {
                    self.reg_pmpcfg[idx_base + 3] = cfg3;
                }
            },
            // zig fmt: off
            .pmpaddr0,  .pmpaddr1,  .pmpaddr2,  .pmpaddr3,  .pmpaddr4,  .pmpaddr5,  .pmpaddr6,  .pmpaddr7,
            .pmpaddr8,  .pmpaddr9,  .pmpaddr10, .pmpaddr11, .pmpaddr12, .pmpaddr13, .pmpaddr14, .pmpaddr15,
            .pmpaddr16, .pmpaddr17, .pmpaddr18, .pmpaddr19, .pmpaddr20, .pmpaddr21, .pmpaddr22, .pmpaddr23,
            .pmpaddr24, .pmpaddr25, .pmpaddr26, .pmpaddr27, .pmpaddr28, .pmpaddr29, .pmpaddr30, .pmpaddr31,
            .pmpaddr32, .pmpaddr33, .pmpaddr34, .pmpaddr35, .pmpaddr36, .pmpaddr37, .pmpaddr38, .pmpaddr39,
            .pmpaddr40, .pmpaddr41, .pmpaddr42, .pmpaddr43, .pmpaddr44, .pmpaddr45, .pmpaddr46, .pmpaddr47,
            .pmpaddr48, .pmpaddr49, .pmpaddr50, .pmpaddr51, .pmpaddr52, .pmpaddr53, .pmpaddr54, .pmpaddr55,
            .pmpaddr56, .pmpaddr57, .pmpaddr58, .pmpaddr59, .pmpaddr60, .pmpaddr61, .pmpaddr62, .pmpaddr63,
            // zig fmt: on
            => {
                const idx = csr - @intFromEnum(Registers.pmpaddr0);
                if (self.reg_pmpcfg[idx].l == 1) {
                    // if the PMP entry is locked, not to rewrite
                    return;
                }
                if (idx < 63 and self.reg_pmpcfg[idx + 1].l == 0 and self.reg_pmpcfg[idx + 1].a == 1) {
                    // if PMP entry i is locked and pmpicfg.A is set to TOR, writes to pmpaddri-1 are ignored.
                    return;
                }
                log.debug("[CSR.WRITE] {} val=0x{x}", .{ c, val });
                self.reg_pmpaddr[idx] = val;
            },
            .mnscratch, .mnepc, .mncause, .mnstatus => {
                log.warn("Smrnmi extension is not supported", .{});
                return CPUError.IllegalInstruction;
            },
            .stvec => {
                self.reg_stvec = @bitCast(val);
                log.debug("[CSR.WRITE] stvec write {}", .{self.reg_stvec});
            },
            .sie => {
                const sie: RegMie = @bitCast(val);
                self.reg_mie.ssie = sie.ssie;
                self.reg_mie.stie = sie.stie;
                self.reg_mie.seie = sie.seie;
                self.reg_mie.lcofie = sie.lcofie;
                log.debug("[CSR.WRITE] sie write {}", .{self.reg_mie});
            },
            .sepc => {
                self.reg_sepc = val;
                log.debug("[CSR.WRITE] sepc write 0x{x}", .{self.reg_sepc});
            },
            .scause => {
                self.reg_scause = @bitCast(val);
                log.debug("[CSR.WRITE] scause write {}", .{self.reg_scause});
            },
            .stval => {
                self.reg_stval = val;
                log.debug("[CSR.WRITE] stval write 0x{x}", .{self.reg_stval});
            },
            .sscratch => {
                self.reg_sscratch = val;
                log.debug("[CSR.WRITE] sscrach write 0x{x}", .{self.reg_sscratch});
            },
            .sip => {
                const sip: RegMip = @bitCast(val);
                self.reg_mip.ssip = sip.ssip;
                self.reg_mip.stip = sip.stip;
                self.reg_mip.seip = sip.seip;
                self.reg_mip.lcofip = sip.lcofip;
                log.debug("[CSR.WRITE] sip write {}", .{self.reg_mip});
            },
            .sstatus => {
                // sstatus is a subset of mstatus
                const ss: RegSstatus = @bitCast(val);
                self.reg_mstatus.sie = ss.sie;
                self.reg_mstatus.spie = ss.spie;
                self.reg_mstatus.ube = ss.ube;
                self.reg_mstatus.spp = ss.spp;
                self.reg_mstatus.sum = ss.sum;
                self.reg_mstatus.mxr = ss.mxr;
                self.reg_mstatus.sd = ss.sd;

                self.reg_mstatus.vs = 0;
                self.reg_mstatus.fs = 0;
                self.reg_mstatus.xs = 0;
                log.debug("[CSR.WRITE] sstatus write {}", .{self.reg_mstatus});
            },
            .satp => {
                self.reg_satp = @bitCast(val);
                self.sv32_enabled = self.reg_satp.mode == 1;
                log.debug("[CSR.WRITE] satp {}", .{self.reg_satp});
            },
            else => {
                log.warn("TODO: write to CSR {} (0x{x}) is not handled", .{ c, csr });
            },
        }
    }

    // phy_addr must be 4 bytes aligned
    fn check_memory_access(self: Self, phy_addr: u34, r: bool, w: bool, x: bool) !void {
        _ = self;
        _ = phy_addr;
        _ = r;
        _ = w;
        _ = x;

        // TODO: implement this
        return;
    }
};

const Sv32VirtualAddress = packed struct(u32) {
    offset: u12,
    vpn0: u10,
    vpn1: u10,
};

const Sv32PhysicalAddress = packed struct(u34) {
    offset: u12 = 0,
    ppn0: u10 = 0,
    ppn1: u12 = 0,
};

// X W R Meaning
// 0 0 0 Pointer to next level of page table.
// 0 0 1 Read-only page.
// 0 1 0 Reserved for future use.
// 0 1 1 Read-write page.
// 1 0 0 Execute-only page.
// 1 0 1 Read-execute page.
// 1 1 0 Reserved for future use.
// 1 1 1 Read-write-execute page
const Sv32PageTableEntry = packed struct(u32) {
    v: u1,
    r: u1,
    w: u1,
    x: u1,
    u: u1,
    g: u1,
    a: u1,
    d: u1,
    rsw: u2,
    ppn0: u10,
    ppn1: u12,
};

const MemoryAccessType = enum {
    Instruction,
    Load,
    Store,
    AMO,

    const Self = @This();
    fn to_page_fault(t: Self) !void {
        switch (t) {
            .Instruction => return CPUError.InstructionPageFault,
            .Load => return CPUError.LoadPageFault,
            .Store => return CPUError.StorePageFault,
            .AMO => return CPUError.AMOPageFault,
        }
    }
};

pub const CPUError = error{
    TooLargeMemoryData,
    OutOfMemoryArea,
    InvalidCSR,
    ReadOnlyCSR,
    InvalidAlignment,
    EcallInvoked,
    InvalidCSRState,
    NotImplemented,
    UARTHooked,

    SupervisorSoftwareInterrupt,
    MachineSoftwareInterrupt,
    SupervisorTimerInterrupt,
    MachineTimerInterrupt,
    SupervisorExternalInterrupt,
    MachineExternalInterrupt,
    CounterOverflowInterrupt,

    InstructionAddressMisaligned,
    IllegalInstruction,
    EnvironmentBreak,
    LoadAddressMisaligned,
    StoreAddressMisaligned,
    AMOAddressMisaligned,
    EnvironmentCallFromUmode,
    EnvironmentCallFromSmode,
    EnvironmentCallFromMmode,
    InstructionPageFault,
    LoadPageFault,
    StoreAMOPageFault,
};

pub fn to_exception_code(e: usize) u31 {
    return switch (e) {
        @intFromError(CPUError.SupervisorSoftwareInterrupt) => 1,
        @intFromError(CPUError.MachineSoftwareInterrupt) => 3,
        @intFromError(CPUError.SupervisorTimerInterrupt) => 5,
        @intFromError(CPUError.MachineTimerInterrupt) => 7,
        @intFromError(CPUError.SupervisorExternalInterrupt) => 9,
        @intFromError(CPUError.MachineExternalInterrupt) => 11,
        @intFromError(CPUError.InstructionAddressMisaligned) => 0,
        @intFromError(CPUError.IllegalInstruction) => 2,
        @intFromError(CPUError.EnvironmentBreak) => 3,
        @intFromError(CPUError.LoadAddressMisaligned) => 4,
        @intFromError(CPUError.StoreAddressMisaligned) => 6,
        @intFromError(CPUError.AMOAddressMisaligned) => 6,
        @intFromError(CPUError.EnvironmentCallFromUmode) => 8,
        @intFromError(CPUError.EnvironmentCallFromSmode) => 9,
        @intFromError(CPUError.EnvironmentCallFromMmode) => 11,
        @intFromError(CPUError.InstructionPageFault) => 12,
        @intFromError(CPUError.LoadPageFault) => 13,
        @intFromError(CPUError.StoreAMOPageFault) => 15,
        else => 0,
    };
}

const MMIOError = error{
    InvalidWordSize,
    NotHooked,
};

const CLINT = struct {
    csr: *CSR,
    cycle_counter: u64 = 0,
    mtime: u64 = 0,
    mtimecmp: u64 = 0,

    const Self = @This();
    const CLINT_BASE: u32 = 0x2000000;
    const CLINT_MSIP_ADDR: u32 = Self.CLINT_BASE;
    const CLINT_MTIMECMP_ADDR: u32 = Self.CLINT_BASE + 0x4000;
    const CLINT_MTIME_ADDR: u32 = Self.CLINT_BASE + 0xBFF8;

    pub fn init(csr: *CSR) Self {
        return .{
            .csr = csr,
        };
    }

    pub fn mem_read(self: *Self, addr: u34, word_size: usize) MMIOError!WORD {
        if (word_size != 4) return MMIOError.InvalidWordSize;
        switch (addr) {
            CLINT_MTIMECMP_ADDR => return @as(WORD, @intCast(self.mtimecmp & 0xFFFF_FFFF)),
            CLINT_MTIMECMP_ADDR + 4 => return @as(WORD, @intCast((self.mtimecmp >> 32) & 0xFFFF_FFFF)),
            CLINT_MTIME_ADDR => return @as(WORD, @intCast(self.mtime & 0xFFFF_FFFF)),
            CLINT_MTIME_ADDR + 4 => return @as(WORD, @intCast((self.mtime >> 32) & 0xFFFF_FFFF)),
            else => return MMIOError.NotHooked,
        }
    }

    pub fn mem_write(self: *Self, addr: u34, val: WORD, word_size: usize) MMIOError!void {
        if (word_size != 4) return MMIOError.InvalidWordSize;
        switch (addr) {
            CLINT_MTIMECMP_ADDR => self.mtimecmp = self.mtimecmp & 0xFFFFFFFF_00000000 | val,
            CLINT_MTIMECMP_ADDR + 4 => self.mtimecmp = (@as(u64, @intCast(val)) << 32) | self.mtimecmp & 0xFFFFFFFF,
            CLINT_MTIME_ADDR => self.mtime = self.mtime & 0xFFFFFFFF_00000000 | val,
            CLINT_MTIME_ADDR + 4 => self.mtime = (@as(u64, @intCast(val)) << 32) | self.mtime & 0xFFFFFFFF,
            else => return MMIOError.NotHooked,
        }
    }

    pub fn tick(self: *Self) void {
        self.cycle_counter += 1;
        if (self.cycle_counter % 100 == 0) {
            self.mtime += 1;
        }

        if (self.mtime >= self.mtimecmp) {
            self.csr.reg_mip.mtip = 1;
        } else {
            self.csr.reg_mip.mtip = 0;
        }
    }
};

const UART = struct {
    csr: *CSR,

    // TODO: implement TX FIFO
    fcr: u8 = 0,
    lcr: u8 = 0,
    isr: u8 = 0,
    ier: u8 = 0,

    const Self = @This();
    const UART_BASE: u32 = 0x10000000;

    // http://byterunner.com/16550.html
    const UART_RHR_ADDR: u32 = UART_BASE;
    const UART_THR_ADDR: u32 = UART_BASE;
    const UART_IER_ADDR: u32 = UART_BASE + 1 * 4; // intrrupt enable register
    const UART_FCR_ADDR: u32 = UART_BASE + 2 * 4; // FIFO control register
    const UART_ISR_ADDR: u32 = UART_BASE + 2 * 4; // interrupt status register
    const UART_LCR_ADDR: u32 = UART_BASE + 3 * 4; // line control register
    const UART_LSR_ADDR: u32 = UART_BASE + 5 * 4; // line status register

    const FCR_FIFO_ENABLE: u32 = 1;
    const FCR_FIFO_CLEAR: u32 = 3 << 1;
    const LCR_EIGHT_BITS: u32 = 3;
    const LCR_BAUD_LATCH: u32 = 1 << 7;
    const LSR_RX_READY: u32 = 1;
    const LSR_TX_IDLE: u32 = 1 << 5;

    pub fn init(csr: *CSR) Self {
        return .{
            .csr = csr,
        };
    }

    pub fn mem_read(self: *Self, addr: u34, word_size: usize) MMIOError!WORD {
        if (word_size != 1) return MMIOError.InvalidWordSize;
        _ = self;
        switch (addr) {
            UART_RHR_ADDR => return 0,
            UART_ISR_ADDR => return 0,
            UART_LSR_ADDR => return LSR_TX_IDLE, // TX is already idle.
            else => return MMIOError.NotHooked,
        }
    }

    pub fn mem_write(self: *Self, addr: u34, val: WORD, word_size: usize) MMIOError!void {
        if (word_size != 1) return MMIOError.InvalidWordSize;
        switch (addr) {
            UART_THR_ADDR => std.debug.print("{c}", .{@as(u8, @intCast(val & 0xFF))}),
            UART_IER_ADDR => self.ier = @intCast(val & 0xFF),
            UART_FCR_ADDR => self.fcr = @intCast(val & 0xFF),
            UART_LCR_ADDR => self.lcr = @intCast(val & 0xFF),
            else => return MMIOError.NotHooked,
        }
    }
};

pub const CPU = struct {
    pc: WORD,
    regs: [32]WORD,
    mem: [MEMORY_SIZE >> 2]WORD,
    mem_reserves: std.AutoHashMap(WORD, void),
    csr: CSR,
    clint: CLINT,
    uart: UART,
    exit_on_ecall: bool = false,

    excep_next_pc: WORD = 0,
    excep_vaddr: WORD = 0,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator) Self {
        var csr = CSR.init();
        const clint = CLINT.init(&csr);
        const uart = UART.init(&csr);
        return .{
            .pc = MEMORY_BASE_ADDR, // for riscv-tests
            .regs = [1]WORD{0} ** 32,
            .mem = [1]WORD{0} ** (MEMORY_SIZE >> 2),
            .mem_reserves = std.AutoHashMap(WORD, void).init(allocator),
            .csr = csr,
            .clint = clint,
            .uart = uart,
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

    pub fn mem_read(self: *Self, addr: u34, exec: bool, word_size: usize) !WORD {
        var epriv_lvl = self.csr.current_level;
        if ((!exec) and self.csr.reg_mstatus.mprv == 1) {
            epriv_lvl = @enumFromInt(self.csr.reg_mstatus.mpp);
        }

        std.log.debug("[MEM.READ] 0x{x} EffectivePrivilege = {}", .{ addr, epriv_lvl });
        if (epriv_lvl == .Machine or !self.csr.sv32_enabled) {
            try self.csr.check_memory_access((addr >> 2) << 2, true, false, exec);
            return try self.mem_pread(addr, exec, word_size);
        } else {
            const phy_addr = try self.sv32_translate_vaddr_to_paddr(@intCast(addr & (0xFFFF_FFFF)), epriv_lvl, true, false, exec);
            return try self.mem_pread(phy_addr, exec, word_size);
        }
    }

    pub fn mem_write(self: *Self, addr: u34, val: WORD, word_size: usize) !void {
        var epriv_lvl = self.csr.current_level;
        if (self.csr.reg_mstatus.mprv == 1) {
            epriv_lvl = @enumFromInt(self.csr.reg_mstatus.mpp);
        }

        std.log.debug("[MEM.WRITE] 0x{x} EffectivePrivilege = {}", .{ addr, epriv_lvl });
        if (epriv_lvl == .Machine or !self.csr.sv32_enabled) {
            try self.mem_pwrite(addr, val, word_size);
        } else {
            const phy_addr = try self.sv32_translate_vaddr_to_paddr(@intCast(addr & (0xFFFF_FFFF)), epriv_lvl, false, true, false);
            try self.mem_pwrite(phy_addr, val, word_size);
        }
    }

    pub fn mem_pread(self: *Self, addr: u34, exec: bool, word_size: usize) !WORD {
        try self.csr.check_memory_access((addr >> 2) << 2, true, false, exec);

        if (self.clint.mem_read(addr, word_size)) |res| {
            return res;
        } else |_| {} // if not hooked, continue.
        if (self.uart.mem_read(addr, word_size)) |res| {
            return res;
        } else |_| {} // if not hooked, continue.

        // TODO: remove MEMORY_BASE_ADDR
        const mem_addr = (addr - MEMORY_BASE_ADDR) >> 2;
        const lower_addr: u2 = @intCast(addr & 0x3);
        switch (word_size) {
            1 => switch (lower_addr) {
                0b00 => return self.mem[mem_addr] & 0xFF,
                0b01 => return (self.mem[mem_addr] >> 8) & 0xFF,
                0b10 => return (self.mem[mem_addr] >> 16) & 0xFF,
                0b11 => return (self.mem[mem_addr] >> 24) & 0xFF,
            },
            2 => switch (lower_addr) {
                0b00 => return self.mem[mem_addr] & 0xFFFF,
                0b01 => return (self.mem[mem_addr] >> 8) & 0xFFFF,
                0b10 => return (self.mem[mem_addr] >> 16) & 0xFFFF,
                0b11 => return ((self.mem[mem_addr + 1] & 0xFF) << 8) | ((self.mem[mem_addr] >> 24) & 0xFF),
            },
            4 => switch (lower_addr) {
                0b00 => return self.mem[mem_addr],
                0b01 => return ((self.mem[mem_addr + 1] & 0xFF) << 24) | (self.mem[mem_addr] >> 8),
                0b10 => return ((self.mem[mem_addr + 1] & 0xFFFF) << 16) | (self.mem[mem_addr] >> 16),
                0b11 => return ((self.mem[mem_addr + 1] & 0xFFFFFF) << 8) | (self.mem[mem_addr] >> 24),
            },
            else => return MMIOError.InvalidWordSize,
        }
    }

    pub fn mem_pwrite(self: *Self, addr: u34, val: WORD, word_size: usize) !void {
        try self.csr.check_memory_access((addr >> 2) << 2, false, true, false);

        if (self.clint.mem_write(addr, val, word_size)) {
            return;
        } else |_| {}
        if (self.uart.mem_write(addr, val, word_size)) |res| {
            return res;
        } else |_| {} // if not hooked, continue.

        // TODO: remove MEMORY_BASE_ADDR
        const mem_addr = (addr - MEMORY_BASE_ADDR) >> 2;
        const lower_addr: u2 = @intCast(addr & 0x3);
        switch (word_size) {
            1 => switch (lower_addr) {
                0b00 => self.mem[mem_addr] = (self.mem[mem_addr] & 0xFFFFFF00) | (val & 0xFF),
                0b01 => self.mem[mem_addr] = (self.mem[mem_addr] & 0xFFFF00FF) | ((val & 0xFF) << 8),
                0b10 => self.mem[mem_addr] = (self.mem[mem_addr] & 0xFF00FFFF) | ((val & 0xFF) << 16),
                0b11 => self.mem[mem_addr] = (self.mem[mem_addr] & 0x00FFFFFF) | ((val & 0xFF) << 24),
            },
            2 => switch (lower_addr) {
                0b00 => self.mem[mem_addr] = (self.mem[mem_addr] & 0xFFFF0000) | (val & 0xFFFF),
                0b01 => self.mem[mem_addr] = (self.mem[mem_addr] & 0xFF0000FF) | ((val & 0xFFFF) << 8),
                0b10 => self.mem[mem_addr] = (self.mem[mem_addr] & 0x0000FFFF) | ((val & 0xFFFF) << 16),
                0b11 => {
                    self.mem[mem_addr] = (self.mem[mem_addr] & 0x00FFFFFF) | ((val & 0xFFFF) << 24);
                    self.mem[mem_addr + 1] = (self.mem[mem_addr + 1] & 0xFFFFFF00) | ((val & 0xFFFF) >> 8);
                },
            },
            4 => switch (lower_addr) {
                0b00 => self.mem[mem_addr] = val,
                0b01 => {
                    self.mem[mem_addr] = (self.mem[mem_addr] & 0x000000FF) | (val << 8);
                    self.mem[mem_addr + 1] = (self.mem[mem_addr + 1] & 0xFFFFFF00) | (val >> 24);
                },
                0b10 => {
                    self.mem[mem_addr] = (self.mem[mem_addr] & 0x0000FFFF) | (val << 16);
                    self.mem[mem_addr + 1] = (self.mem[mem_addr + 1] & 0xFFFF0000) | (val >> 16);
                },
                0b11 => {
                    self.mem[mem_addr] = (self.mem[mem_addr] & 0x00FFFFFF) | (val << 24);
                    self.mem[mem_addr + 1] = (self.mem[mem_addr + 1] & 0xFF000000) | (val >> 8);
                },
            },
            else => return MMIOError.InvalidWordSize,
        }

        // for riscv-tests
        if (addr == 0x80001000 or addr == 0x80001004) {
            if (addr == 0x80001000) {
                std.log.err("tohost = {}: {c}", .{ addr - 0x80001000, @as(u8, @intCast(val & 0xFF)) });
            }
            self.mem[mem_addr] = 0;
        }
        return;
    }

    // A virtual address va is translated into a physical address pa as follows:
    // 1. Let a be satp.ppn×PAGESIZE, and let i=LEVELS-1. (For Sv32, PAGESIZE=2^12 and LEVELS=2.) The
    // satp register must be active, i.e., the effective privilege mode must be S-mode or U-mode.
    // 2. Let pte be the value of the PTE at address a+va.vpn[i]×PTESIZE. (For Sv32, PTESIZE=4.) If
    // accessing pte violates a PMA or PMP check, raise an access-fault exception corresponding to the
    // original access type.
    // 3. If pte.v=0, or if pte.r=0 and pte.w=1, or if any bits or encodings that are reserved for future standard
    // use are set within pte, stop and raise a page-fault exception corresponding to the original access
    // type.
    // 4. Otherwise, the PTE is valid. If pte.r=1 or pte.x=1, go to step 5. Otherwise, this PTE is a pointer to the
    // next level of the page table. Let i=i-1. If i<0, stop and raise a page-fault exception corresponding to
    // the original access type. Otherwise, let a=pte.ppn×PAGESIZE and go to step 2.
    // 5. A leaf PTE has been found. Determine if the requested memory access is allowed by the pte.r, pte.w,
    // pte.x, and pte.u bits, given the current privilege mode and the value of the SUM and MXR fields of
    // the mstatus register. If not, stop and raise a page-fault exception corresponding to the original
    // access type.
    // 6. If i>0 and pte.ppn[i-1:0] ≠ 0, this is a misaligned superpage; stop and raise a page-fault exception
    // corresponding to the original access type.
    // 7. If pte.a=0, or if the original memory access is a store and pte.d=0:
    // ◦ If the Svade extension is implemented, stop and raise a page-fault exception corresponding to
    // the original access type.
    // ◦ If a store to pte would violate a PMA or PMP check, raise an access-fault exception
    // corresponding to the original access type.
    // ◦ Perform the following steps atomically:
    // ▪ Compare pte to the value of the PTE at address a+va.vpn[i]×PTESIZE.
    // ▪ If the values match, set pte.a to 1 and, if the original memory access is a store, also set pte.d
    // to 1.
    // ▪ If the comparison fails, return to step 2.
    // 8. The translation is successful. The translated physical address is given as follows:
    // ◦ pa.pgoff = va.pgoff.
    // ◦ If i>0, then this is a superpage translation and pa.ppn[i-1:0] = va.vpn[i-1:0].
    // ◦ pa.ppn[LEVELS-1:i] = pte.ppn[LEVELS-1:i].
    // The results of implicit address-translation reads in step 2 may be held in a read-only, incoherent
    // address-translation cache but not shared with other harts. The address-translation cache may hold an
    // arbitrary number of entries, including an arbitrary number of entries for the same address and ASID.
    // Entries in the address-translation cache may then satisfy subsequent step 2 reads if the ASID
    // associated with the entry matches the ASID loaded in step 0 or if the entry is associated with a global
    // mapping. To ensure that implicit reads observe writes to the same memory locations, an
    // SFENCE.VMA instruction must be executed after the writes to flush the relevant cached translations.
    // The address-translation cache cannot be used in step 7; accessed and dirty bits may only be updated in
    // memory directly.
    pub fn sv32_translate_vaddr_to_paddr(self: *Self, vaddr: u32, priv_lvl: CSR.PrivilegeLevels, read: bool, write: bool, exec: bool) !u34 {
        return self.sv32_translate_vaddr_to_paddr_impl(vaddr, priv_lvl, read, write, exec) catch |err| {
            self.excep_vaddr = vaddr;
            return err;
        };
    }

    pub fn sv32_translate_vaddr_to_paddr_impl(self: *Self, vaddr: u32, priv_lvl: CSR.PrivilegeLevels, read: bool, write: bool, exec: bool) !u34 {
        // step. 0
        const vaddr_sv32 = @as(Sv32VirtualAddress, @bitCast(vaddr));

        // step. 1
        // 1. Let a be satp.ppn×PAGESIZE, and let i=LEVELS-1. (For Sv32, PAGESIZE=2^12 and LEVELS=2.) The
        // satp register must be active, i.e., the effective privilege mode must be S-mode or U-mode.
        var a = @as(u34, @intCast(self.csr.reg_satp.ppn)) * 4096;
        var i: usize = 2 - 1;
        std.log.debug("VA = {}", .{vaddr_sv32});
        while (true) {
            // step. 2
            // 2. Let pte be the value of the PTE at address a+va.vpn[i]×PTESIZE. (For Sv32, PTESIZE=4.) If
            // accessing pte violates a PMA or PMP check, raise an access-fault exception corresponding to the original access type.
            var pte_addr: u34 = 0;
            if (i == 1) {
                pte_addr = a + @as(u34, @intCast(vaddr_sv32.vpn1)) * 4;
            } else {
                pte_addr = a + @as(u34, @intCast(vaddr_sv32.vpn0)) * 4;
            }
            // TODO: handle this
            try self.csr.check_memory_access(pte_addr, read, write, exec);
            var pte = @as(Sv32PageTableEntry, @bitCast(self.mem[(pte_addr - MEMORY_BASE_ADDR) >> 2]));
            log.debug("PTE at 0x{x} = {}", .{ pte_addr, pte });

            // X W R Meaning
            // 0 0 0 Pointer to next level of page table.
            // 0 0 1 Read-only page.
            // 0 1 0 Reserved for future use.
            // 0 1 1 Read-write page.
            // 1 0 0 Execute-only page.
            // 1 0 1 Read-execute page.
            // 1 1 0 Reserved for future use.
            // 1 1 1 Read-write-execute page
            // step. 3
            // 3. If pte.v=0, or if pte.r=0 and pte.w=1, or if any bits or encodings that are reserved for future standard
            // use are set within pte, stop and raise a page-fault exception corresponding to the original access type.
            if (pte.v == 0 or (pte.r == 0 and pte.w == 1)) {
                if (exec) return CPUError.InstructionPageFault;
                if (write) return CPUError.StoreAMOPageFault;
                if (read) return CPUError.LoadPageFault;
            }

            // step. 4
            // 4. Otherwise, the PTE is valid. If pte.r=1 or pte.x=1, go to step 5. Otherwise, this PTE is a pointer to the
            // next level of the page table. Let i=i-1. If i<0, stop and raise a page-fault exception corresponding to
            // the original access type. Otherwise, let a=pte.ppn×PAGESIZE and go to step 2.
            if (!(pte.r == 1 or pte.x == 1)) {
                i = i - 1;
                if (i < 0) {
                    if (exec) return CPUError.InstructionPageFault;
                    if (write) return CPUError.StoreAMOPageFault;
                    if (read) return CPUError.LoadPageFault;
                }
                const ppn: u34 = @as(u34, @intCast(@as(u22, @intCast(pte.ppn1)) << 10 | pte.ppn0));
                a = ppn * 4096;
                continue;
            } else {
                // step. 5
                // 5. A leaf PTE has been found. Determine if the requested memory access is allowed by the pte.r, pte.w,
                // pte.x, and pte.u bits, given the current privilege mode and the value of the SUM and MXR fields of
                // the mstatus register. If not, stop and raise a page-fault exception corresponding to the original
                // access type.

                // The SUM (permit Supervisor User Memory access) bit modifies the privilege with which S-mode loads
                // and stores access virtual memory. When SUM=0, S-mode memory accesses to pages that are
                // accessible by U-mode (U=1 in Figure 59) will fault. When SUM=1, these accesses are permitted.
                if ((priv_lvl == .User and pte.u != 1) or (priv_lvl == .Supervisor and self.csr.reg_mstatus.sum == 0 and pte.u == 1)) {
                    if (exec) return CPUError.InstructionPageFault;
                    if (write) return CPUError.StoreAMOPageFault;
                    if (read) return CPUError.LoadPageFault;
                }

                if (exec and !(pte.r == 1 and pte.x == 1)) return CPUError.InstructionPageFault;
                if (write and !(pte.w == 1 and pte.r == 1)) return CPUError.StoreAMOPageFault;
                if (read and !(pte.r == 1)) {
                    // The MXR (Make eXecutable Readable) bit modifies the privilege with which loads access virtual
                    // memory. When MXR=0, only loads from pages marked readable (R=1 in Figure 59) will succeed.
                    // When MXR=1, loads from pages marked either readable or executable (R=1 or X=1) will succeed. MXR
                    // has no effect when page-based virtual memory is not in effect. MXR is read-only 0 if S-mode is not
                    // supported.
                    if (!(self.csr.reg_mstatus.mxr == 1 and pte.x == 1)) return CPUError.LoadPageFault;
                }

                // step. 6
                // 6. If i>0 and pte.ppn[i-1:0] ≠ 0, this is a misaligned superpage; stop and raise a page-fault exception
                // corresponding to the original access type.
                if (i > 0) {
                    // only i=1 can happen.
                    if (pte.ppn0 != 0) {
                        if (exec) return CPUError.InstructionPageFault;
                        if (write) return CPUError.StoreAMOPageFault;
                        if (read) return CPUError.LoadPageFault;
                    }
                }

                // step. 7
                // 7. If pte.a=0, or if the original memory access is a store and pte.d=0:
                // ◦ If the Svade extension is implemented, stop and raise a page-fault exception corresponding to
                // the original access type.
                // ◦ If a store to pte would violate a PMA or PMP check, raise an access-fault exception
                // corresponding to the original access type.
                // ◦ Perform the following steps atomically:
                // ▪ Compare pte to the value of the PTE at address a+va.vpn[i]×PTESIZE.
                // ▪ If the values match, set pte.a to 1 and, if the original memory access is a store, also set pte.d
                // to 1.
                // ▪ If the comparison fails, return to step 2.
                if (pte.a == 0 or (write and pte.d == 0)) {
                    // TODO: handle error
                    try self.csr.check_memory_access(pte_addr, read, write, exec);

                    // we ensure the PTE is not modified by others
                    pte.a = 1;
                    if (write) pte.d = 1;

                    self.mem[(pte_addr - MEMORY_BASE_ADDR) >> 2] = @bitCast(pte);
                }

                // 8. The translation is successful. The translated physical address is given as follows:
                // ◦ pa.pgoff = va.pgoff.
                // ◦ If i>0, then this is a superpage translation and pa.ppn[i-1:0] = va.vpn[i-1:0].
                // ◦ pa.ppn[LEVELS-1:i] = pte.ppn[LEVELS-1:i].
                var pa = Sv32PhysicalAddress{
                    .offset = vaddr_sv32.offset,
                };
                if (i > 0) {
                    // i = 1
                    pa.ppn0 = vaddr_sv32.vpn0;
                } else {
                    pa.ppn0 = pte.ppn0;
                }
                pa.ppn1 = pte.ppn1;

                const phy_addr: u34 = @bitCast(pa);
                std.log.debug("[Sv32] VA: 0x{x} -> PA: 0x{x}", .{ vaddr, phy_addr });
                return phy_addr;
            }
        }
    }

    pub fn tick_cycle(self: *Self) !void {
        self.tick_cycle_impl() catch |err| switch (err) {
            CPUError.SupervisorSoftwareInterrupt,
            CPUError.MachineSoftwareInterrupt,
            CPUError.SupervisorTimerInterrupt,
            CPUError.MachineTimerInterrupt,
            CPUError.SupervisorExternalInterrupt,
            CPUError.MachineExternalInterrupt,
            CPUError.CounterOverflowInterrupt,
            CPUError.InstructionAddressMisaligned,
            CPUError.IllegalInstruction,
            CPUError.EnvironmentBreak,
            CPUError.LoadAddressMisaligned,
            CPUError.StoreAddressMisaligned,
            CPUError.AMOAddressMisaligned,
            CPUError.EnvironmentCallFromUmode,
            CPUError.EnvironmentCallFromSmode,
            CPUError.EnvironmentCallFromMmode,
            CPUError.InstructionPageFault,
            CPUError.LoadPageFault,
            CPUError.StoreAMOPageFault,
            => {
                var intr = false;
                switch (err) {
                    CPUError.SupervisorSoftwareInterrupt,
                    CPUError.MachineSoftwareInterrupt,
                    CPUError.SupervisorTimerInterrupt,
                    CPUError.MachineTimerInterrupt,
                    CPUError.SupervisorExternalInterrupt,
                    CPUError.MachineExternalInterrupt,
                    CPUError.CounterOverflowInterrupt,
                    => intr = true,
                    else => intr = false,
                }

                const ec = to_exception_code(@intFromError(err));
                var delegate = false;
                if (intr) {
                    delegate = (std.math.shr(u32, @bitCast(self.csr.reg_mideleg), ec) & 0x1) == 1;
                } else {
                    delegate = (std.math.shr(u64, self.csr.reg_medeleg, ec) & 0x1) == 1;
                }
                if (self.csr.current_level != .Machine and delegate) {
                    // When a trap is delegated to S-mode, the scause register is written with the trap cause; the sepc register
                    // is written with the virtual address of the instruction that took the trap; the stval register is written
                    // with an exception-specific datum; the SPP field of mstatus is written with the active privilege mode at
                    // the time of the trap; the SPIE field of mstatus is written with the value of the SIE field at the time of
                    // the trap; and the SIE field of mstatus is cleared. The mcause, mepc, and mtval registers and the MPP
                    // and MPIE fields of mstatus are not written.

                    self.csr.reg_scause = .{ .interrupt = @bitCast(intr), .exception_code = ec };

                    switch (self.csr.current_level) {
                        .User => self.csr.reg_mstatus.spp = 0,
                        .Supervisor => self.csr.reg_mstatus.spp = 1,
                        else => return CPUError.IllegalInstruction,
                    }
                    self.csr.current_level = .Supervisor;

                    switch (err) {
                        CPUError.InstructionAddressMisaligned,
                        CPUError.LoadAddressMisaligned,
                        CPUError.StoreAddressMisaligned,
                        CPUError.AMOAddressMisaligned,
                        => {
                            self.csr.reg_stval = self.excep_next_pc;
                        },
                        CPUError.InstructionPageFault,
                        CPUError.LoadPageFault,
                        CPUError.StoreAMOPageFault,
                        => {
                            self.csr.reg_stval = self.excep_vaddr;
                        },
                        else => {},
                    }

                    self.csr.reg_mstatus.spie = self.csr.reg_mstatus.sie;
                    self.csr.reg_mstatus.sie = 0;

                    // When a trap is taken into M-mode, mepc is written with the virtual address of the instruction that was
                    // interrupted or that encountered the exception. Otherwise, mepc is never written by the
                    // implementation, though it may be explicitly written by software
                    self.csr.reg_sepc = self.pc;
                    switch (self.csr.reg_stvec.mode) {
                        0b00 => {
                            // jump to base directly
                            self.pc = @as(u32, self.csr.reg_stvec.base) << 2;
                        },
                        0b1 => {
                            // jump with offset provided by exception code.
                            self.pc = (@as(u32, self.csr.reg_stvec.base) << 2) + (4 * ec);
                        },
                        else => {
                            return CPUError.InvalidCSRState;
                        },
                    }
                    if (self.exit_on_ecall and (err == CPUError.EnvironmentCallFromUmode or err == CPUError.EnvironmentCallFromSmode or err == CPUError.EnvironmentCallFromMmode)) {
                        return CPUError.EcallInvoked;
                    }
                    log.debug("[STRAP] Exception {} trapped. jumped to 0x{x}", .{ err, self.pc });
                } else {
                    self.csr.reg_mcause = .{ .interrupt = @bitCast(intr), .exception_code = ec };
                    // trap illegal-instruction exception
                    // change level to Machine mode.
                    self.csr.reg_mstatus.mpp = @intFromEnum(self.csr.current_level);
                    self.csr.current_level = .Machine;

                    switch (err) {
                        CPUError.InstructionAddressMisaligned,
                        CPUError.LoadAddressMisaligned,
                        CPUError.StoreAddressMisaligned,
                        CPUError.AMOAddressMisaligned,
                        => {
                            self.csr.reg_mtval = self.excep_next_pc;
                        },
                        CPUError.InstructionPageFault,
                        CPUError.LoadPageFault,
                        CPUError.StoreAMOPageFault,
                        => {
                            self.csr.reg_mtval = self.excep_vaddr;
                        },
                        else => {},
                    }

                    self.csr.reg_mstatus.mpie = self.csr.reg_mstatus.mie;
                    self.csr.reg_mstatus.mie = 0;

                    // When a trap is taken into M-mode, mepc is written with the virtual address of the instruction that was
                    // interrupted or that encountered the exception. Otherwise, mepc is never written by the
                    // implementation, though it may be explicitly written by software
                    self.csr.reg_mepc = self.pc;
                    switch (self.csr.reg_mtvec.mode) {
                        0b0 => {
                            // jump to base directly
                            self.pc = @as(u32, self.csr.reg_mtvec.base) << 2;
                        },
                        0b1 => {
                            // jump with offset provided by exception code.
                            self.pc = (@as(u32, self.csr.reg_mtvec.base) << 2) + (4 * ec);
                        },
                        else => {
                            return CPUError.InvalidCSRState;
                        },
                    }
                    if (self.exit_on_ecall and (err == CPUError.EnvironmentCallFromUmode or err == CPUError.EnvironmentCallFromSmode or err == CPUError.EnvironmentCallFromMmode)) {
                        return CPUError.EcallInvoked;
                    }
                    log.debug("[MTRAP] Exception {} trapped. jumped to 0x{x}", .{ err, self.pc });
                }
            },
            else => {
                return err;
            },
        };
    }

    pub fn tick_cycle_impl(self: *Self) !void {
        if (self.pc & 0x3 != 0) {
            return CPUError.InstructionAddressMisaligned;
        }

        // check interrupts trapped to M-mode
        if ((self.csr.current_level == .Machine and self.csr.reg_mstatus.mie == 1) or self.csr.current_level != .Machine) {
            if (self.csr.reg_mip.meip == 1 and self.csr.reg_mie.meie == 1 and self.csr.reg_mideleg.meip == 0) return CPUError.MachineExternalInterrupt;
            if (self.csr.reg_mip.msip == 1 and self.csr.reg_mie.msie == 1 and self.csr.reg_mideleg.msip == 0) return CPUError.MachineSoftwareInterrupt;
            if (self.csr.reg_mip.mtip == 1 and self.csr.reg_mie.mtie == 1 and self.csr.reg_mideleg.mtip == 0) return CPUError.MachineTimerInterrupt;
            if (self.csr.reg_mip.seip == 1 and self.csr.reg_mie.seie == 1 and self.csr.reg_mideleg.seip == 0) return CPUError.SupervisorExternalInterrupt;
            if (self.csr.reg_mip.ssip == 1 and self.csr.reg_mie.ssie == 1 and self.csr.reg_mideleg.ssip == 0) return CPUError.SupervisorSoftwareInterrupt;
            if (self.csr.reg_mip.stip == 1 and self.csr.reg_mie.stie == 1 and self.csr.reg_mideleg.stip == 0) return CPUError.SupervisorTimerInterrupt;
        }

        // check interrupts trapped to S-mode
        if ((self.csr.current_level == .Supervisor and self.csr.reg_mstatus.sie == 1) or self.csr.current_level == .User) {
            if (self.csr.reg_mip.seip == 1 and self.csr.reg_mie.seie == 1) return CPUError.SupervisorExternalInterrupt;
            if (self.csr.reg_mip.ssip == 1 and self.csr.reg_mie.ssie == 1) return CPUError.SupervisorSoftwareInterrupt;
            if (self.csr.reg_mip.stip == 1 and self.csr.reg_mie.stie == 1) return CPUError.SupervisorTimerInterrupt;
        }

        self.clint.tick();

        // instructions are stored in 4 bytes aligned
        const inst = try self.mem_read(self.pc, true, 4);
        log.debug("MEM[0x{x}] INST=0b{b} (0x{x})", .{ self.pc, inst, inst });
        var ecall_exit = false;
        errdefer {
            if (!ecall_exit) {
                std.log.warn("Illegal instruction at MEM[0x{x}] INST=0b{b} (0x{x})", .{ self.pc, inst, inst });
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

                switch (funct3) {
                    0b000, 0b100 => {
                        const mem_val = try self.mem_read(mem_addr, false, 1);
                        if (funct3 == 0b000) {
                            log.debug("LB rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = sign_ext(mem_val, 8);
                        } else {
                            log.debug("LBU rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = mem_val;
                        }
                    },
                    0b001, 0b101 => {
                        const mem_val = try self.mem_read(mem_addr, false, 2);
                        if (funct3 == 0b001) {
                            log.debug("LH rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = sign_ext(mem_val, 16);
                        } else {
                            log.debug("LHU rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                            self.regs[rd] = mem_val;
                        }
                    },
                    0b010 => {
                        const mem_val = try self.mem_read(mem_addr, false, 4);
                        log.debug("LW rd={} rs1={} offset=0x{x} addr=0x{x} val=0x{x}", .{ rd, rs1, offset, mem_addr, mem_val });
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
                const rs2_val = self.read_reg(rs2);

                switch (funct3) {
                    0b000 => {
                        log.debug("SB rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                        try self.mem_write(mem_addr, rs2_val, 1);
                    },
                    0b001 => {
                        log.debug("SH rd={} rs1={} offset=0x{x} addr=0x{x}", .{ rd, rs1, offset, mem_addr });
                        try self.mem_write(mem_addr, rs2_val, 2);
                    },
                    0b010 => {
                        log.debug("SW rd={} rs1={} rs2={} offset=0x{x} addr=0x{x} val=0x{x}", .{ rd, rs1, rs2, offset, mem_addr, rs2_val });
                        try self.mem_write(mem_addr, rs2_val, 4);
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
                const next_pc = @addWithOverflow(self.pc, imm)[0]; // adding any values including negative one
                // validate next PC is aligned in 4 bytes.
                if (next_pc & 0x3 != 0) {
                    // If mtval is written with a nonzero value when a misaligned load or store causes an access-fault or
                    // page-fault exception, then mtval will contain the virtual address of the portion of the access that
                    // caused the fault.
                    self.excep_next_pc = next_pc;
                    return CPUError.InstructionAddressMisaligned;
                }

                self.regs[rd] = @addWithOverflow(self.pc, 4)[0];
                self.pc = next_pc;
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

                // LSB is set to zero.
                const next_pc = @addWithOverflow(rs1_val, imm)[0] & (0xFFFFFFFE);
                // validate next PC is aligned in 4 bytes.
                if (next_pc & 0x3 != 0) {
                    // If mtval is written with a nonzero value when a misaligned load or store causes an access-fault or
                    // page-fault exception, then mtval will contain the virtual address of the portion of the access that
                    // caused the fault.
                    self.excep_next_pc = next_pc;
                    return CPUError.InstructionAddressMisaligned;
                }

                self.regs[rd] = (self.pc + 4);
                self.pc = next_pc;
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
                        switch (funct7) {
                            0b0000000 => {
                                self.regs[rd] = @addWithOverflow(rs1_val, rs2_val)[0];
                                log.debug("ADD rd={} rs1={} rs2={} (0x{x}= 0x{x} + 0x{x})", .{ rd, rs1, rs2, self.regs[rd], rs1_val, rs2_val });
                            },
                            0b0100000 => {
                                log.debug("SUB rd={} rs1={} 0x{x} rs2={} 0x{x}", .{ rd, rs1, rs1_val, rs2, rs2_val });
                                self.regs[rd] = @subWithOverflow(rs1_val, rs2_val)[0];
                            },
                            0b0000001 => {
                                log.debug("MUL rd={} rs1={} 0x{x} rs2={} 0x{x}", .{ rd, rs1, rs1_val, rs2, rs2_val });
                                self.regs[rd] = @mulWithOverflow(rs1_val, rs2_val)[0];
                            },
                            else => {
                                return CPUError.IllegalInstruction;
                            },
                        }
                    },
                    0b001 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("SLL rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                // rs1 by the shift amount held in the lower 5 bits of register rs2.
                                self.regs[rd] = std.math.shl(WORD, self.read_reg(rs1), self.read_reg(rs2) & 0x1F);
                            },
                            0b0000001 => {
                                log.debug("MULH rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                const rs1_vali: i64 = @intCast(@as(i32, @bitCast(rs1_val)));
                                const rs2_vali: i64 = @intCast(@as(i32, @bitCast(rs2_val)));
                                const res_i64 = @mulWithOverflow(rs1_vali, rs2_vali)[0];
                                self.regs[rd] = @intCast((@as(u64, @bitCast(res_i64)) >> 32));
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    0b010 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("SLT rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                if (@as(i32, @bitCast(rs1_val)) < @as(i32, @bitCast(rs2_val))) {
                                    self.regs[rd] = 1;
                                } else {
                                    self.regs[rd] = 0;
                                }
                            },
                            0b0000001 => {
                                log.debug("MULHSU rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                const rs1_vali: i64 = @intCast(@as(i32, @bitCast(rs1_val)));
                                const rs2_vali: i64 = @intCast(rs2_val); // this is treated as unsigned value
                                const res_i64 = @mulWithOverflow(rs1_vali, rs2_vali)[0];
                                self.regs[rd] = @intCast((@as(u64, @bitCast(res_i64)) >> 32));
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    0b011 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("SLTU rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                if (rs1_val < rs2_val) {
                                    self.regs[rd] = 1;
                                } else {
                                    self.regs[rd] = 0;
                                }
                            },
                            0b0000001 => {
                                log.debug("MULH rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                const rs1_vall: u64 = @intCast(rs1_val);
                                const rs2_vall: u64 = @intCast(rs2_val);
                                const res = @mulWithOverflow(rs1_vall, rs2_vall)[0];
                                self.regs[rd] = @intCast(res >> 32);
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    0b100 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("XOR rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                self.regs[rd] = rs1_val ^ rs2_val;
                            },
                            0b0000001 => {
                                log.debug("DIV rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                var res: i32 = -1;
                                const rs1_vali: i32 = @bitCast(rs1_val);
                                const rs2_vali: i32 = @bitCast(rs2_val);
                                //  Table 11. Semantics for division by zero and division overflow.
                                if (rs2_val == 0) {} else if (rs1_vali == -0x80000000 and rs2_vali == -1) {
                                    res = rs1_vali;
                                } else {
                                    res = @divTrunc(rs1_vali, rs2_vali);
                                }
                                self.regs[rd] = @bitCast(res);
                            },
                            else => {
                                return CPUError.IllegalInstruction;
                            },
                        }
                    },
                    0b101 => {
                        const shamt: u5 = @intCast(rs2_val & 0x1F);
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("SRL rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                self.regs[rd] = rs1_val >> shamt;
                            },
                            0b0100000 => {
                                log.debug("SRA rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                const bit_width = @bitSizeOf(WORD) - @as(WORD_BIT_WIDTH_PLUS_ONE, shamt);
                                log.debug("shamt={} bit_width={}", .{ shamt, bit_width });
                                self.regs[rd] = sign_ext(rs1_val >> shamt, bit_width);
                            },
                            0b0000001 => {
                                log.debug("DIVU rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                if (rs2_val == 0) {
                                    self.regs[rd] = 0xFFFF_FFFF;
                                } else {
                                    self.regs[rd] = @divTrunc(rs1_val, rs2_val);
                                }
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    0b110 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("OR rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                self.regs[rd] = rs1_val | rs2_val;
                            },
                            0b0000001 => {
                                log.debug("REM rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                const rs1_vali: i32 = @bitCast(rs1_val);
                                const rs2_vali: i32 = @bitCast(rs2_val);
                                var res: i32 = rs1_vali;
                                if (rs2_val == 0) {} else if (rs1_vali == -0x80000000 and rs2_vali == -1) {
                                    res = 0;
                                } else {
                                    res = @rem(rs1_vali, rs2_vali);
                                }
                                self.regs[rd] = @bitCast(res);
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    0b111 => {
                        switch (funct7) {
                            0b0000000 => {
                                log.debug("AND rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                self.regs[rd] = rs1_val & rs2_val;
                            },
                            0b0000001 => {
                                log.debug("REMU rd={} rs1={} rs2={}", .{ rd, rs1, rs2 });
                                var res = rs1_val;
                                if (rs2_val != 0) {
                                    res = @rem(rs1_val, rs2_val);
                                }
                                self.regs[rd] = res;
                            },
                            else => return CPUError.IllegalInstruction,
                        }
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
                        switch (inst >> 25) {
                            0b0000000 => {
                                log.debug("SLLI rd={} rs1={} shamt={}", .{ rd, rs1, shamt });
                                self.regs[rd] = self.read_reg(rs1) << shamt;
                            },
                            else => return CPUError.IllegalInstruction,
                        }
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
                        switch (inst >> 25) {
                            0b0000000 => {
                                log.debug("SRLI rd={} rs1={} shamt={}", .{ rd, rs1, shamt });
                                self.regs[rd] = self.read_reg(rs1) >> shamt;
                            },
                            0b0100000 => {
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
                var jump = false;
                switch (funct3) {
                    0b000 => {
                        log.debug("BEQ rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        jump = self.read_reg(rs1) == self.read_reg(rs2);
                    },
                    0b001 => {
                        log.debug("BNE rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        jump = self.read_reg(rs1) != self.read_reg(rs2);
                    },
                    0b100 => {
                        const rs1_val: i32 = @bitCast(self.read_reg(rs1));
                        const rs2_val: i32 = @bitCast(self.read_reg(rs2));
                        log.debug("BLT rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        jump = rs1_val < rs2_val;
                    },
                    0b101 => {
                        const rs1_val: i32 = @bitCast(self.read_reg(rs1));
                        const rs2_val: i32 = @bitCast(self.read_reg(rs2));
                        log.debug("BGE rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        jump = rs1_val >= rs2_val;
                    },
                    0b110 => {
                        log.debug("BLTU rs1={} rs2={} imm={}", .{ rs1, rs2, imm });
                        jump = self.read_reg(rs1) < self.read_reg(rs2);
                    },
                    0b111 => {
                        const rs1_val = self.read_reg(rs1);
                        const rs2_val = self.read_reg(rs2);
                        log.debug("BGEU rs1={}(0x{x}) rs2={}(0x{x}) imm={}", .{ rs1, rs1_val, rs2, rs2_val, imm });
                        jump = rs1_val >= rs2_val;
                    },
                    else => {
                        return CPUError.IllegalInstruction;
                    },
                }
                if (jump) {
                    const next_pc = @addWithOverflow(self.pc, imm)[0]; // adding any values including negative one
                    // validate next PC is aligned in 4 bytes.
                    if (next_pc & 0x3 != 0) {
                        // If mtval is written with a nonzero value when a misaligned load or store causes an access-fault or
                        // page-fault exception, then mtval will contain the virtual address of the portion of the access that
                        // caused the fault.
                        self.excep_next_pc = next_pc;
                        return CPUError.InstructionAddressMisaligned;
                    }

                    self.pc = next_pc;
                    return;
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
                    switch (self.csr.current_level) {
                        CSR.PrivilegeLevels.Machine => return CPUError.EnvironmentCallFromMmode,
                        CSR.PrivilegeLevels.Supervisor => return CPUError.EnvironmentCallFromSmode,
                        CSR.PrivilegeLevels.User => return CPUError.EnvironmentCallFromUmode,
                        else => return CPUError.IllegalInstruction,
                    }
                } else if (inst == 0x100073) {
                    log.debug("EBREAK", .{});
                    log.warn("TODO: implement EBREAK", .{});
                    return CPUError.EnvironmentBreak;
                } else {
                    switch (funct3) {
                        0b000 => {
                            if (rd == 0 and rs1 == 0 and csr == 0b001100000010) {
                                // An MRET or SRET instruction is used to return from a trap in M-mode or S-mode respectively. When
                                // executing an xRET instruction, supposing xPP holds the value y, xIE is set to xPIE; the privilege mode is
                                // changed to y; xPIE is set to 1; and xPP is set to the least-privileged supported mode (U if U-mode is
                                // implemented, else M). If y≠M, xRET also sets MPRV=0.
                                const next_priv = @as(CSR.PrivilegeLevels, @enumFromInt(self.csr.reg_mstatus.mpp));
                                self.csr.reg_mstatus.mie = self.csr.reg_mstatus.mpie;
                                self.csr.current_level = next_priv;
                                self.csr.reg_mstatus.mpie = 1;
                                self.csr.reg_mstatus.mpp = @intFromEnum(CSR.PrivilegeLevels.User);

                                // An MRET or SRET instruction that changes the privilege mode to a mode less privileged than M also sets MPRV=0.
                                if (self.csr.current_level != next_priv and self.csr.current_level != .Machine) {
                                    self.csr.reg_mstatus.mprv = 0;
                                }
                                self.pc = self.csr.reg_mepc;
                                log.debug("MRET to 0x{x} (Privilege Level = {})", .{ self.pc, self.csr.current_level });
                                return;
                            } else if (rd == 0 and rs1 == 0 and csr == 0b000100000010) {
                                // The TSR (Trap SRET) bit is a WARL field that supports intercepting the supervisor exception return
                                // instruction, SRET. When TSR=1, attempts to execute SRET while executing in S-mode will raise an
                                // illegal-instruction exception. When TSR=0, this operation is permitted in S-mode. TSR is read-only 0
                                // when S-mode is not supported.
                                if (self.csr.current_level == .Supervisor and self.csr.reg_mstatus.tsr == 1) return CPUError.IllegalInstruction;

                                const next_priv = @as(CSR.PrivilegeLevels, @enumFromInt(@as(u2, @intCast(self.csr.reg_mstatus.spp))));
                                self.csr.reg_mstatus.sie = self.csr.reg_mstatus.spie;
                                self.csr.current_level = next_priv;
                                self.csr.reg_mstatus.spie = 1;
                                self.csr.reg_mstatus.spp = 0; // PrivilegeLevels.User

                                // An MRET or SRET instruction that changes the privilege mode to a mode less privileged than M also sets MPRV=0.
                                if (self.csr.current_level != next_priv and self.csr.current_level != .Machine) {
                                    self.csr.reg_mstatus.mprv = 0;
                                }
                                self.pc = self.csr.reg_sepc;
                                log.debug("SRET to 0x{x} (Privilege Level = {})", .{ self.pc, self.csr.current_level });
                                return;
                            } else if (rd == 0 and rs1 == 0 and csr == 0b000100000101) {
                                log.debug("WFI (Privilege Level = {})", .{self.csr.current_level});

                                // The TW (Timeout Wait) bit is a WARL field that supports intercepting the WFI instruction (see Section 3.3.3).
                                // When TW=0, the WFI instruction may execute in lower privilege modes when not
                                // prevented for some other reason. When TW=1, then if WFI is executed in any less-privileged mode,
                                // and it does not complete within an implementation-specific, bounded time limit, the WFI instruction
                                // causes an illegal-instruction exception. An implementation may have WFI always raise an illegalinstruction exception in less-privileged modes when TW=1, even if there are pending globally-disabled
                                // interrupts when the instruction is executed. TW is read-only 0 when there are no modes less
                                // privileged than M
                                if (self.csr.reg_mstatus.tw == 1 and self.csr.current_level != .Machine) {
                                    // TODO: check timeout?
                                    log.debug("mstatus = {}", .{self.csr.reg_mstatus});
                                    return CPUError.IllegalInstruction;
                                }

                                // When S-mode is implemented, then executing WFI in U-mode causes an illegal-instruction exception,
                                // unless it completes within an implementation-specific, bounded time limit. A future revision of this
                                // specification might add a feature that allows S-mode to selectively permit WFI in U-mode. Such a
                                // feature would only be active when TW=0.
                                if (self.csr.current_level == .User) {
                                    return CPUError.IllegalInstruction;
                                }

                                // Hence, a legal implementation is to simply implement the WFI instruction as a NOP
                                // Nothing to do.
                            } else if (funct7 == 0b0001001) {
                                log.debug("SFENCE.VMA rs1={} rs2={}", .{ rs1, rs2 });
                                log.debug("TODO: implement sfence.vma", .{});
                                // When TVM=1, attempts to read or write the satp CSR or execute an
                                // SFENCE.VMA or SINVAL.VMA instruction while executing in S-mode will raise an illegal-instruction
                                // exception. When TVM=0, these operations are permitted in S-mode. TVM is read-only 0 when Smode is not supported.
                                if (self.csr.current_level == .Supervisor and self.csr.reg_mstatus.tvm == 1) {
                                    return CPUError.IllegalInstruction;
                                }
                            }
                        },
                        0b001 => {
                            log.debug("CSRRW rd={} rs1={} csr=0x{x}", .{ rd, rs1, csr });
                            try CSR.check_writable(csr);
                            const rs1_val = self.read_reg(rs1);
                            if (rd != 0) {
                                const csr_val = try self.csr.read(csr);
                                self.regs[rd] = csr_val;
                            }
                            try self.csr.write(csr, rs1_val);
                        },
                        0b010 => {
                            log.debug("CSRRS rd={} rs1={} csr=0x{x}", .{ rd, rs1, csr });
                            if (rs1 != 0) try CSR.check_writable(csr);
                            const rs1_val = self.read_reg(rs1);
                            const csr_val = try self.csr.read(csr);
                            self.regs[rd] = csr_val;
                            if (rs1 != 0) try self.csr.write(csr, csr_val | rs1_val);
                        },
                        0b011 => {
                            log.debug("CSRRC rd={} rs1={} csr=0x{x}", .{ rd, rs1, csr });
                            if (rs1 != 0) try CSR.check_writable(csr);
                            const rs1_val = self.read_reg(rs1);
                            const csr_val = try self.csr.read(csr);
                            self.regs[rd] = csr_val;
                            if (rs1 != 0) try self.csr.write(csr, csr_val & (~rs1_val));
                        },
                        0b101 => {
                            const uimm = rs1;
                            log.debug("CSRRWI rd={} uimm={} csr=0x{x}", .{ rd, uimm, csr });
                            try CSR.check_writable(csr);
                            if (rd != 0) {
                                const csr_val = try self.csr.read(csr);
                                self.regs[rd] = csr_val;
                            }
                            try self.csr.write(csr, @intCast(uimm));
                        },
                        0b110 => {
                            const uimm = rs1;
                            log.debug("CSRRSI rd={} uimm={} csr=0x{x}", .{ rd, uimm, csr });
                            if (uimm != 0) try CSR.check_writable(csr);
                            const csr_val = try self.csr.read(csr);
                            self.regs[rd] = csr_val;
                            if (uimm != 0) try self.csr.write(csr, csr_val | uimm);
                        },
                        0b111 => {
                            const uimm: WORD = @intCast(rs1);
                            log.debug("CSRRCI rd={} uimm={} csr=0x{x}", .{ rd, uimm, csr });
                            if (uimm != 0) try CSR.check_writable(csr);
                            const csr_val = try self.csr.read(csr);
                            self.regs[rd] = csr_val;
                            if (uimm != 0) try self.csr.write(csr, csr_val & (~uimm));
                        },
                        else => {
                            return CPUError.IllegalInstruction;
                        },
                    }
                }
            },
            0b0101111 => {
                const funct5 = inst >> 27;
                const aq = inst >> 26 & 0x1;
                const rl = inst >> 25 & 0x1;
                switch (funct3) {
                    0b010 => {
                        // load a data value from the address in rs1, place the value into register rd,
                        // apply a binary operator to the loaded value and the original value in rs2,
                        // then store the result back to the original address in rs1.
                        // If the address is not naturally aligned, an address-misaligned exception or an access-fault exception will be generated.
                        // The access-fault exception can be generated for a memory access that would otherwise be able to
                        // complete except for the misalignment, if the misaligned access should not be emulated
                        const mem_addr = self.read_reg(rs1);
                        // memory address must be aligned
                        if (mem_addr & 0x3 != 0) {
                            return CPUError.AMOAddressMisaligned;
                        }
                        const rs2_val = self.read_reg(rs2);
                        const mem_val = try self.mem_read(mem_addr, false, 4);
                        if (funct5 != 0b00010 and funct5 != 0b00011) {
                            // not LR.W and SC.W
                            self.regs[rd] = mem_val;
                        }

                        switch (funct5) {
                            0b00000 => {
                                log.debug("AMOADD.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                try self.mem_write(mem_addr, @addWithOverflow(mem_val, rs2_val)[0], 4);
                            },
                            0b00001 => {
                                log.debug("AMOSWAP.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                try self.mem_write(mem_addr, rs2_val, 4);
                            },
                            0b00100 => {
                                log.debug("AMOXOR.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                try self.mem_write(mem_addr, mem_val ^ rs2_val, 4);
                            },
                            0b01100 => {
                                log.debug("AMOAND.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                try self.mem_write(mem_addr, mem_val & rs2_val, 4);
                            },
                            0b01000 => {
                                log.debug("AMOOR.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                try self.mem_write(mem_addr, mem_val | rs2_val, 4);
                            },
                            0b10000 => {
                                log.debug("AMOMIN.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                if (@as(i32, @bitCast(mem_val)) < @as(i32, @bitCast(rs2_val))) {
                                    try self.mem_write(mem_addr, mem_val, 4);
                                } else {
                                    try self.mem_write(mem_addr, rs2_val, 4);
                                }
                            },
                            0b10100 => {
                                log.debug("AMOMAX.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                if (@as(i32, @bitCast(mem_val)) > @as(i32, @bitCast(rs2_val))) {
                                    try self.mem_write(mem_addr, mem_val, 4);
                                } else {
                                    try self.mem_write(mem_addr, rs2_val, 4);
                                }
                            },
                            0b11000 => {
                                log.debug("AMOMINU.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                if (mem_val < rs2_val) {
                                    try self.mem_write(mem_addr, mem_val, 4);
                                } else {
                                    try self.mem_write(mem_addr, rs2_val, 4);
                                }
                            },
                            0b11100 => {
                                log.debug("AMOMAXU.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                if (mem_val > rs2_val) {
                                    try self.mem_write(mem_addr, mem_val, 4);
                                } else {
                                    try self.mem_write(mem_addr, rs2_val, 4);
                                }
                            },
                            0b00010 => {
                                log.debug("LR.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                if (rs2 != 0) {
                                    return CPUError.IllegalInstruction;
                                }
                                // LR.W loads a word from the address in rs1,
                                // places the sign-extended value in rd, and registers a reservation set—a set of bytes that subsumes the
                                // bytes in the addressed word.

                                // TODO: no need to sign extend?
                                self.regs[rd] = mem_val;
                                try self.mem_reserves.put(mem_addr, void{});
                            },
                            0b00011 => {
                                log.debug("SC.W rd={} rs1={} rs2={} aq={} rl={}", .{ rd, rs1, rs2, aq, rl });
                                // SC.W conditionally writes a word in rs2 to the address in rs1: the SC.W
                                // succeeds only if the reservation is still valid and the reservation set contains the bytes being written. If
                                // the SC.W succeeds, the instruction writes the word in rs2 to memory, and it writes zero to rd. If the
                                // SC.W fails, the instruction does not write to memory, and it writes a nonzero value to rd. For the
                                // purposes of memory protection, a failed SC.W may be treated like a store. Regardless of success or
                                // failure, executing an SC.W instruction invalidates any reservation held by this hart.

                                // TODO: is this OK?
                                if (self.mem_reserves.get(mem_addr)) |_| {
                                    try self.mem_write(mem_addr, rs2_val, 4);
                                    self.regs[rd] = 0;
                                } else {
                                    self.regs[rd] = 1;
                                }
                                self.mem_reserves.clearRetainingCapacity();
                            },
                            else => return CPUError.IllegalInstruction,
                        }
                    },
                    else => return CPUError.IllegalInstruction,
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
        "rv32um-p-div.bin",
        "rv32um-p-divu.bin",
        "rv32um-p-mul.bin",
        "rv32um-p-mulh.bin",
        "rv32um-p-mulhsu.bin",
        "rv32um-p-mulhu.bin",
        "rv32um-p-rem.bin",
        "rv32um-p-remu.bin",
        "rv32ua-p-amoadd_w.bin",
        "rv32ua-p-amoand_w.bin",
        "rv32ua-p-amomaxu_w.bin",
        "rv32ua-p-amomax_w.bin",
        "rv32ua-p-amominu_w.bin",
        "rv32ua-p-amomin_w.bin",
        "rv32ua-p-amoor_w.bin",
        "rv32ua-p-amoswap_w.bin",
        "rv32ua-p-amoxor_w.bin",
        "rv32ua-p-lrsc.bin",
        "rv32mi-p-breakpoint.bin",
        "rv32mi-p-csr.bin",
        "rv32mi-p-illegal.bin",
        "rv32mi-p-lh-misaligned.bin",
        "rv32mi-p-lw-misaligned.bin",
        "rv32mi-p-ma_addr.bin",
        "rv32mi-p-ma_fetch.bin",
        "rv32mi-p-mcsr.bin",
        "rv32mi-p-sbreak.bin",
        "rv32mi-p-scall.bin",
        "rv32mi-p-shamt.bin",
        "rv32mi-p-sh-misaligned.bin",
        "rv32mi-p-sw-misaligned.bin",
        "rv32mi-p-zicntr.bin",
        "rv32si-p-csr.bin",
        "rv32si-p-dirty.bin",
        "rv32si-p-ma_fetch.bin",
        "rv32si-p-sbreak.bin",
        "rv32si-p-scall.bin",
        "rv32si-p-wfi.bin",
    };
    const test_v_files = [_][]const u8{
        "rv32ui-v-add.bin",
        "rv32ui-v-addi.bin",
        "rv32ui-v-and.bin",
        "rv32ui-v-andi.bin",
        "rv32ui-v-auipc.bin",
        "rv32ui-v-beq.bin",
        "rv32ui-v-bge.bin",
        "rv32ui-v-bgeu.bin",
        "rv32ui-v-blt.bin",
        "rv32ui-v-bltu.bin",
        "rv32ui-v-bne.bin",
        "rv32ui-v-fence_i.bin",
        "rv32ui-v-jal.bin",
        "rv32ui-v-jalr.bin",
        "rv32ui-v-lb.bin",
        "rv32ui-v-lbu.bin",
        "rv32ui-v-lh.bin",
        "rv32ui-v-lhu.bin",
        "rv32ui-v-lui.bin",
        "rv32ui-v-lw.bin",
        "rv32ui-v-ma_data.bin",
        "rv32ui-v-or.bin",
        "rv32ui-v-ori.bin",
        "rv32ui-v-sb.bin",
        "rv32ui-v-sh.bin",
        "rv32ui-v-simple.bin",
        "rv32ui-v-sll.bin",
        "rv32ui-v-slli.bin",
        "rv32ui-v-slt.bin",
        "rv32ui-v-slti.bin",
        "rv32ui-v-sltiu.bin",
        "rv32ui-v-sltu.bin",
        "rv32ui-v-sra.bin",
        "rv32ui-v-srai.bin",
        "rv32ui-v-srl.bin",
        "rv32ui-v-srli.bin",
        "rv32ui-v-sub.bin",
        "rv32ui-v-sw.bin",
        "rv32ui-v-xor.bin",
        "rv32ui-v-xori.bin",
        "rv32um-v-div.bin",
        "rv32um-v-divu.bin",
        "rv32um-v-mul.bin",
        "rv32um-v-mulh.bin",
        "rv32um-v-mulhsu.bin",
        "rv32um-v-mulhu.bin",
        "rv32um-v-rem.bin",
        "rv32um-v-remu.bin",
        "rv32ua-v-amoadd_w.bin",
        "rv32ua-v-amoand_w.bin",
        "rv32ua-v-amomaxu_w.bin",
        "rv32ua-v-amomax_w.bin",
        "rv32ua-v-amominu_w.bin",
        "rv32ua-v-amomin_w.bin",
        "rv32ua-v-amoor_w.bin",
        "rv32ua-v-amoswap_w.bin",
        "rv32ua-v-amoxor_w.bin",
        "rv32ua-v-lrsc.bin",
    };
    // zig fmt: on
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const test_dir = try std.fs.cwd().openDir("./riscv-tests/isa", .{});
    for (test_files) |test_file| {
        var f = try test_dir.openFile(test_file, .{});
        const fstat = try f.stat();
        var test_file_buffer = try gpa.allocator().alloc(u8, fstat.size);
        const read_size = try f.readAll(test_file_buffer);
        try std.testing.expectEqual(fstat.size, read_size);

        std.debug.print("testing {s} (size={d})\n", .{ test_file, read_size });

        var c = CPU.init(gpa.allocator());
        c.exit_on_ecall = true;
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

        try std.testing.expectEqual(1, c.read_reg(3));
    }

    for (test_v_files) |test_file| {
        var f = try test_dir.openFile(test_file, .{});
        const fstat = try f.stat();
        var test_file_buffer = try gpa.allocator().alloc(u8, fstat.size);
        const read_size = try f.readAll(test_file_buffer);
        try std.testing.expectEqual(fstat.size, read_size);

        std.debug.print("testing {s} (size={d})\n", .{ test_file, read_size });

        var c = CPU.init(gpa.allocator());
        c.exit_on_ecall = true;
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

        try std.testing.expectEqual(1, c.read_reg(10));
    }
}
