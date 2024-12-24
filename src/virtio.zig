const std = @import("std");
const log = std.log;

pub fn Virtqueue(comptime Parent: type) type {
    return struct {
        descs: []DescriptorTable,
        avail: AvailableRing,
        used: UsedRing,
        last_avail_idx: u16 = 0,

        pub const DescriptorTable = extern struct {
            address: u64 = 0,
            length: u32 = 0,
            flags: u16 = 0,
            next: u16 = 0,
        };
        pub const AvailableRing = struct {
            flags: *u16,
            index: *u16,
            ring: []u16,
            event_idx: *u16,
        };
        pub const UsedRing = struct {
            flags: *u16,
            index: *u16,
            ring: []Ring,
            avail_event: *u16,
            pub const Ring = extern struct {
                index: u32,
                length: u32,
            };
        };

        pub const Error = error{
            TooManyDescriptorsToHandle,
            InvalidRequest,
            InvalidDescriptor,
            NotEnoughDescriptors,
        };
        const Self = @This();

        pub fn init(queue_size: usize, desc_addr: usize, avail_addr: usize, used_addr: usize) Self {
            std.log.info("[virtio] init desc=0x{x} avail=0x{x} used=0x{x}", .{ desc_addr, avail_addr, used_addr });
            const descs = @as([*]DescriptorTable, @ptrFromInt(desc_addr))[0..queue_size];

            var avail = AvailableRing{
                .flags = @ptrFromInt(avail_addr),
                .index = @ptrFromInt(avail_addr + 2),
                .ring = @as([*]u16, @ptrFromInt(avail_addr + 4))[0..queue_size],
                .event_idx = @ptrFromInt(avail_addr + 4 + (2 * queue_size)),
            };
            avail.ring.len = queue_size;

            var used = UsedRing{
                .flags = @ptrFromInt(used_addr),
                .index = @ptrFromInt(used_addr + 2),
                .ring = @as([*]UsedRing.Ring, @ptrFromInt(used_addr + 4))[0..queue_size],
                .avail_event = @ptrFromInt(used_addr + 4 + (8 * queue_size)),
            };
            used.ring.len = queue_size;

            return .{
                .descs = descs,
                .avail = avail,
                .used = used,
            };
        }

        pub fn handle(self: *Self, parent: *Parent) !bool {
            const ar = &self.avail;
            const ur = &self.used;
            var descs_handle: [64]*DescriptorTable = undefined;

            //self.queue.print();
            while (ar.index.* != self.last_avail_idx) {
                var desc_num: usize = 0;
                var descID = ar.ring[self.last_avail_idx % ar.ring.len];
                ur.ring[ur.index.* % ur.ring.len].length = 0;
                ur.ring[ur.index.* % ur.ring.len].index = descID;
                while (true) {
                    if (desc_num >= descs_handle.len) {
                        return Error.TooManyDescriptorsToHandle;
                    }
                    const desc = &self.descs[descID];

                    descs_handle[desc_num] = desc;
                    desc_num += 1;

                    ur.ring[ur.index.* % ur.ring.len].length += desc.length;

                    if ((desc.flags & 0x1) == 0) {
                        break;
                    }

                    descID = desc.next;
                }

                try parent.handleVirtqueue(descs_handle[0..desc_num]);

                self.last_avail_idx = @addWithOverflow(self.last_avail_idx, 1)[0];
                ur.index.* = @addWithOverflow(ur.index.*, 1)[0];
            }

            return ur.flags.* & 0x1 != 0x1;
        }
    };
}
//pub fn VirtioQueue(comptime size: usize, comptime Parent: type) type {
//    return struct {
//        queue: *Queue,
//        last_avail_idx: u16 = 0,
//        mem_addr: u64,
//
//        pub const QueueSize = size;
//        pub const Queue = VirtioQueueSplit(size);
//        const Self = @This();
//
//        pub fn init(addr: u64, mem_addr: u64) Self {
//            return .{
//                .queue = @intToPtr(*Queue, addr),
//                .mem_addr = mem_addr,
//            };
//        }
//
//        const Error = error{
//            TooManyDescriptorsToHandle,
//            InvalidRequest,
//            InvalidDescriptor,
//            NotEnoughDescriptors,
//        };
//
//        // @return: inject IRQ or not
//        pub fn handle(self: *Self, parent: *Parent) !bool {
//            const descs = &self.queue.descriptor_tables;
//            const ar = &self.queue.available_ring;
//            const ur = &self.queue.used_ring;
//            var descs_handle: [64]*Queue.DescriptorTable = undefined;
//
//            //self.queue.print();
//            while (ar.index != self.last_avail_idx) {
//                var desc_num: usize = 0;
//                var descID = ar.ring[self.last_avail_idx % size];
//                ur.ring[ur.index % ur.ring.len].length = 0;
//                ur.ring[ur.index % ur.ring.len].index = descID;
//                while (true) {
//                    if (desc_num >= descs_handle.len) {
//                        return Error.TooManyDescriptorsToHandle;
//                    }
//                    const desc = &descs[descID];
//
//                    descs_handle[desc_num] = desc;
//                    desc_num += 1;
//
//                    ur.ring[ur.index % ur.ring.len].length += desc.length;
//
//                    if ((desc.flags & 0x1) == 0) {
//                        break;
//                    }
//
//                    descID = desc.next;
//                }
//
//                try parent.handleVirtioQueue(descs_handle[0..desc_num]);
//
//                self.last_avail_idx = @addWithOverflow(self.last_avail_idx, 1)[0];
//                ur.index = @addWithOverflow(ur.index, 1)[0];
//            }
//
//            return ur.flags & 0x1 != 0x1;
//        }
//
//        pub fn put(self: *Self, buf: []const u8) !bool {
//            const descs = &self.queue.descriptor_tables;
//            const ar = &self.queue.available_ring;
//            const ur = &self.queue.used_ring;
//            var rest_len = buf.len;
//            const cur_last_avail_idx = self.last_avail_idx;
//            var prev_descID: i32 = -1;
//            while (rest_len > 0) {
//                if (self.last_avail_idx == ar.index) {
//                    log.errExt(@src(), "queue is full", .{});
//                    self.last_avail_idx = cur_last_avail_idx;
//                    return Error.NotEnoughDescriptors;
//                }
//                const descID = ar.ring[self.last_avail_idx % QueueSize];
//                const desc = &descs[descID];
//
//                if (rest_len == buf.len) {
//                    ur.ring[ur.index % QueueSize].index = descID;
//                    ur.ring[ur.index % QueueSize].length = 0;
//                }
//
//                var desc_buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
//                const desc_len = @intCast(u32, @min(rest_len, desc.length));
//                const buf_idx = buf.len - rest_len;
//                std.mem.copy(u8, desc_buf, buf[buf_idx .. buf_idx + desc_len]);
//                desc.length = desc_len;
//                ur.ring[ur.index % QueueSize].length += desc_len;
//
//                if (prev_descID != -1) {
//                    const p = @intCast(usize, prev_descID);
//                    descs[p].flags |= 0x1; // desc is chained
//                    descs[p].next = descID;
//                }
//
//                prev_descID = descID;
//                self.last_avail_idx = @addWithOverflow(self.last_avail_idx, 1)[0];
//                rest_len -= desc_len;
//            }
//
//            ur.index = @addWithOverflow(ur.index, 1)[0];
//
//            return ur.flags & 0x1 != 0x1;
//        }
//    };
//}
//

//fn VirtioQueueSplit(comptime size: usize) type {
//    return extern struct {
//        descriptor_tables: [QueueSize]DescriptorTable = [_]DescriptorTable{.{}} ** QueueSize,
//        available_ring: AvailableRing = .{},
//        // 4KiB paging alignment
//        _padding: [4096 - (((@sizeOf(DescriptorTable) * QueueSize) + @sizeOf(AvailableRing)) % 4096)]u8 = undefined,
//        used_ring: UsedRing = .{},
//
//        const Self = @This();
//        pub const QueueSize = size;
//
//
//        pub fn print(self: Self) void {
//            const descs = self.descriptor_tables;
//            const ar = self.available_ring;
//            const ur = self.used_ring;
//            log.debug("VirtioQueue", .{});
//            var i: usize = 0;
//            while (i < self.descriptor_tables.len) : (i += 1) {
//                log.debug("- Desc[{d}] = (addr:0x{x} len:0x{x} flags:0x{x} next:0x{x})", .{ i, descs[i].address, descs[i].length, descs[i].flags, descs[i].next });
//            }
//            log.debug("- AvailRing = (flags:0x{x} index:0x{x} event_idx:0x{x})", .{ ar.flags, ar.index, ar.event_idx });
//            i = 0;
//            while (i < ar.ring.len) : (i += 1) {
//                log.debug("  - Ring[{d}] = 0x{x}", .{ i, ar.ring[i] });
//            }
//            log.debug("- UsedRing = (flags:0x{x} index:0x{x} avail_event:0x{x})", .{ ur.flags, ur.index, ur.avail_event });
//            i = 0;
//            while (i < ur.ring.len) : (i += 1) {
//                log.debug("  - Ring[{d}] = index:0x{x} length:0x{x}", .{ i, ur.ring[i].index, ur.ring[i].length });
//            }
//        }
//    };
//}
