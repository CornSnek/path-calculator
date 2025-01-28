//!Getting little-endian bytes to get the factorial total of pathfinder algorithms to interact with js.
//!Also used to get boolean combinations of bits.
const std = @import("std");
const ProgressNumber = @This();
bytes: std.ArrayListUnmanaged(u8),
pub fn deinit(self: *ProgressNumber, allocator: std.mem.Allocator) void {
    self.bytes.deinit(allocator);
}
pub fn init(allocator: std.mem.Allocator, init_num: u8) !ProgressNumber {
    var bytes: std.ArrayListUnmanaged(u8) = .{};
    try bytes.append(allocator, init_num);
    return .{ .bytes = bytes };
}
pub fn init_int(allocator: std.mem.Allocator, init_num: anytype) !ProgressNumber {
    const IntType = @typeInfo(@TypeOf(init_num));
    if (IntType != .Int) @compileError("init_num should be an integer");
    var bytes: std.ArrayListUnmanaged(u8) = .{};
    const init_num_little = std.mem.nativeToLittle(@TypeOf(init_num), init_num);
    try bytes.appendNTimes(allocator, undefined, IntType.Int.bits / 8);
    @memcpy(bytes.items, std.mem.asBytes(&init_num_little)[0 .. IntType.Int.bits / 8]);
    return .{ .bytes = bytes };
}
pub fn clone(self: ProgressNumber, allocator: std.mem.Allocator) !ProgressNumber {
    return .{ .bytes = try self.bytes.clone(allocator) };
}
pub fn add_one(self: *ProgressNumber, allocator: std.mem.Allocator) !void {
    for (0..self.bytes.items.len) |i| {
        self.bytes.items[i], const carry = @addWithOverflow(self.bytes.items[i], 1);
        if (carry == 0) {
            return;
        } else {
            if (i == self.bytes.items.len - 1) {
                try self.bytes.append(allocator, 1);
                return;
            }
        }
    }
}
pub fn add(self: *ProgressNumber, allocator: std.mem.Allocator, by_pn: ProgressNumber) !void {
    if (by_pn.bytes.items.len > self.bytes.items.len)
        try self.bytes.appendNTimes(allocator, 0, by_pn.bytes.items.len - self.bytes.items.len);
    var carry: u1 = 0;
    for (0..self.bytes.items.len) |i| {
        self.bytes.items[i], carry = @addWithOverflow(self.bytes.items[i], carry);
        const by_pn_byte = if (i < by_pn.bytes.items.len) by_pn.bytes.items[i] else 0; //0 if out of bounds. self may be repeated bytes of FF so carry will keep adding...
        self.bytes.items[i], const carry2 = @addWithOverflow(self.bytes.items[i], by_pn_byte);
        carry |= carry2; //Case 1: carry made it overflow from 0xFF to 0x00, making carry2 always 0. Case 2: carry did not overflow, and carry2 might
    }
    if (carry == 1)
        try self.bytes.append(allocator, 1);
}
///Multiply small numbers
pub fn multiply_byte(self: *ProgressNumber, allocator: std.mem.Allocator, by: u8) !void {
    var carry: u8 = 0;
    for (0..self.bytes.items.len) |i| {
        const product: u16 = @as(u16, @intCast(self.bytes.items[i])) * by + carry;
        self.bytes.items[i] = @intCast(product & 0xFF);
        carry = @intCast(product >> 8);
    }
    if (carry != 0)
        try self.bytes.append(allocator, carry);
}
///Switch back and forth in adding/multiplying
const MultiplyResult = union(enum) {
    init: ProgressNumber,
    sum_second: [2]ProgressNumber,
    sum_first: [2]ProgressNumber,
    fn deinit(self: *MultiplyResult, allocator: std.mem.Allocator) void {
        switch (self.*) {
            .init => |*pn| pn.deinit(allocator),
            inline else => |*arr| {
                arr[0].deinit(allocator);
                arr[1].deinit(allocator);
            },
        }
    }
};
pub fn multiply(self: *ProgressNumber, allocator: std.mem.Allocator, by_pn: ProgressNumber) !void {
    var mr: MultiplyResult = .{ .init = try self.clone(allocator) };
    errdefer mr.deinit(allocator);
    try mr.init.multiply_byte(allocator, by_pn.bytes.items[0]);
    for (1..by_pn.bytes.items.len) |i| {
        const by_pn_byte = by_pn.bytes.items[i];
        switch (mr) {
            .init => |*pn| {
                mr = .{ .sum_second = .{ pn.*, try self.clone(allocator) } };
                try mr.sum_second[1].shift_bytes_right(allocator, i);
                try mr.sum_second[1].multiply_byte(allocator, by_pn_byte);
            },
            .sum_second => |*pn_arr| {
                try pn_arr[1].add(allocator, pn_arr[0]);
                pn_arr[0].bytes.clearRetainingCapacity();
                try pn_arr[0].bytes.appendSlice(allocator, self.bytes.items);
                try pn_arr[0].shift_bytes_right(allocator, i);
                try pn_arr[0].multiply_byte(allocator, by_pn_byte);
                mr = .{ .sum_first = pn_arr.* };
            },
            .sum_first => |*pn_arr| {
                try pn_arr[0].add(allocator, pn_arr[1]);
                pn_arr[1].bytes.clearRetainingCapacity();
                try pn_arr[1].bytes.appendSlice(allocator, self.bytes.items);
                try pn_arr[1].shift_bytes_right(allocator, i);
                try pn_arr[1].multiply_byte(allocator, by_pn_byte);
                mr = .{ .sum_second = pn_arr.* };
            },
        }
    }
    switch (mr) {
        .init => |*pn| {
            self.deinit(allocator);
            self.bytes = pn.bytes;
        },
        .sum_first => |*pn_arr| {
            try pn_arr[0].add(allocator, pn_arr[1]);
            pn_arr[1].deinit(allocator);
            self.deinit(allocator);
            self.bytes = pn_arr[0].bytes;
        },
        .sum_second => |*pn_arr| {
            try pn_arr[1].add(allocator, pn_arr[0]);
            pn_arr[0].deinit(allocator);
            self.deinit(allocator);
            self.bytes = pn_arr[1].bytes;
        },
    }
}
///Pad with zeroes
pub fn pad(self: *ProgressNumber, allocator: std.mem.Allocator, by_bytes: usize) !void {
    if (by_bytes > self.bytes.items.len) {
        try self.bytes.ensureTotalCapacityPrecise(allocator, by_bytes);
        const old_len: usize = self.bytes.items.len;
        self.bytes.items.len = by_bytes;
        for (old_len..self.bytes.items.len) |i| {
            self.bytes.items[i] = 0;
        }
    }
}
pub fn bit(self: ProgressNumber, offset: usize) bool {
    return self.bytes.items[offset / 8] & (@as(u8, 1) << @as(u3, @intCast(offset % 8))) != 0;
}
pub fn set(self: ProgressNumber, offset: usize) void {
    self.bytes.items[offset / 8] |= (@as(u8, 1) << @as(u3, @intCast(offset % 8)));
}
pub fn shift_bytes_right(self: *ProgressNumber, allocator: std.mem.Allocator, by: usize) !void {
    const old_len = self.bytes.items.len;
    try self.bytes.appendNTimes(allocator, undefined, by);
    std.mem.copyBackwards(u8, self.bytes.items[by..], self.bytes.items[0..old_len]);
    @memset(self.bytes.items[0..by], 0);
}
pub fn format(self: ProgressNumber, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
    try writer.writeAll("{ ");
    for (0..self.bytes.items.len) |i| {
        const b = self.bytes.items[i];
        try std.fmt.formatIntValue(b, fmt, options, writer);
        if (i != self.bytes.items.len - 1) try writer.writeAll(", ");
    }
    try writer.writeAll(" }");
}
