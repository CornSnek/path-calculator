const std = @import("std");
const nodes = @import("nodes.zig");
pub const std_options: std.Options = .{
    .logFn = @import("logger.zig").std_options_impl.logFn,
};
const ExampleMap =
    \\01 02 03 04 05
    \\06 07 s  9v 10
    \\11 12 13 14 15
    \\16 17 18 19 20
    \\c
    \\3,1 3,2 3,3 2,2 4,2
;
const ExampleMap2 =
    \\225 125 130 100 130 125 130 250 130 100 130 100 225 100 400
    \\100 130 100 130 100 400 100 75  100 130 100 50  100 130 250
    \\130 100 130 100 130 100 130 50  130 100 130 100 130 100 130
    \\250 130 125 130 100 130 100 75  100 130 225 250 100 130 50 
    \\400 100 130 100 130 125 100 75  100 100 130 100 130 100 130
    \\100 130 100 130 100 75  100 75  100 75  100 130 100 130 100
    \\130 100 130 100 100 100 100 75  100 100 100 100 50  100 225
    \\75  75  125 75  75  75  75  s   50v 50v 75  75  75  75  75 
    \\250 100 130 100 100 100 100 75  100 100 100 100 130 100 400
    \\100 130 100 130 100 75  100 75  100 75  100 130 100 130 225
    \\130 150 225 100 130 150 100 75  225 100 130 100 225 100 130
    \\100 130 100 130 100 130 100 75  225 100 130 100 225 100 130
    \\130 100 130 100 250 100 130 75  130 100 130 100 130 225 130
    \\150 130 100 130 100 400 150 75  100 130 225 130 100 130 225
    \\400 100 130 150 130 100 130 75  250 100 225 100 130 100 400
    \\c
    \\0,0 4,0
    \\12,1
    \\8,2 13,2
    \\3,3 9,3
    \\10,4
    \\0,5 2,5
    \\1,8
    \\13,9
    \\0,10 3,10
    \\9,12
    \\7,13 12,13
;
const ExampleMap3 =
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 s   100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\100 100 100 100 100 100 100 100 100 100 100 100 100 100 100
    \\c
    \\9,10 8,11 9,11 10,11 9,12
;
pub fn main() !void {
    var GPA: std.heap.GeneralPurposeAllocator(.{}) = .{};
    defer _ = GPA.deinit();
    const allocator = GPA.allocator();
    var nmap = try nodes.NodeMap.init(allocator, ExampleMap3);
    defer nmap.deinit(allocator);
    std.log.debug("LCN pathfind:\n", .{});
    try nmap.mcn_path(allocator);
    std.log.debug("SSN pathfind:\n", .{});
    try nmap.ssn_path(allocator);
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
