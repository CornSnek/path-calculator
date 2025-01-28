//!Getting little-endian bytes to get the factorial total of the brute force algorithm to interact with js.
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
///Only 0-255 just to multiply small numbers for factorials
pub fn multiply(self: *ProgressNumber, allocator: std.mem.Allocator, by: u8) !void {
    var carry: u8 = 0;
    for (0..self.bytes.items.len) |i| {
        const product: u16 = @as(u16, @intCast(self.bytes.items[i])) * by + carry;
        self.bytes.items[i] = @intCast(product & 0xFF);
        carry = @intCast(product >> 8);
    }
    if (carry != 0)
        try self.bytes.append(allocator, carry);
}
///Pad with zeroes
pub fn expand(self: *ProgressNumber, allocator: std.mem.Allocator, by_bytes: usize) !void {
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
pub fn format(self: ProgressNumber, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
    try writer.writeAll("{ ");
    for (0..self.bytes.items.len) |i| {
        const b = self.bytes.items[i];
        try std.fmt.formatIntValue(b, fmt, options, writer);
        if (i != self.bytes.items.len - 1) try writer.writeAll(", ");
    }
    try writer.writeAll(" }");
}
