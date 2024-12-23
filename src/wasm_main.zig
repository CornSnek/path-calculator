const std = @import("std");
const nodes = @import("nodes.zig");
const jsalloc = @import("wasm_jsalloc.zig");
const WasmError = @import("wasm_print.zig").WasmError;
const FlushPrint = @import("wasm_print.zig").FlushPrint;
const logger = @import("logger.zig");
pub const std_options: std.Options = .{
    .logFn = logger.std_options_impl.logFn,
};
pub const panic = @import("wasm_print.zig").panic;
pub const allocator = std.heap.wasm_allocator;
usingnamespace jsalloc;
export fn TestFn() void {
    std.log.debug("Abcd\n", .{});
}

export fn FindPath(grid_str: [*c]const u8, size: usize) void {
    std.debug.assert(grid_str != 0);
    const grid_slice = grid_str[0..size];
    std.log.debug("{s}", .{grid_slice});
    find_path(grid_slice) catch |e| WasmError(e);
    FlushPrint();
}

//Pathfinder byte format: {Combined Path Total Cost, Total number of X paths, X1..., X2... }
//X format: {Bytes to read (Excluding this byte), Start x, Start y, End x, End y, Path Total Cost, Number of D Directions, D1..., D2... }
//D format: {Cost of direction, nodes.NodeMap.Direction enum}
pub var PathfinderArrayList: std.ArrayListUnmanaged(u32) = .{};
fn find_path(grid_slice: []const u8) !void {
    PathfinderArrayList.clearRetainingCapacity();
    var nmap = try nodes.NodeMap.init(allocator, grid_slice);
    defer nmap.deinit(allocator);
    try nmap.get_shortest_path(allocator);
    ParsePathfinder(PathfinderArrayList.items.ptr, PathfinderArrayList.items.len);
}
extern fn ParsePathfinder([*c]u32, usize) void;
