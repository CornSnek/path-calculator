const std = @import("std");
const nodes = @import("nodes.zig");
const jsalloc = @import("wasm_jsalloc.zig");
const wasm_print = @import("wasm_print.zig");
const WasmError = wasm_print.WasmError;
const FlushPrint = wasm_print.FlushPrint;
const logger = @import("logger.zig");
pub const std_options: std.Options = .{
    .logFn = logger.std_options_impl.logFn,
};
pub const panic = wasm_print.panic;
pub const allocator = std.heap.wasm_allocator;
comptime {
    std.mem.doNotOptimizeAway(jsalloc.WasmAlloc);
    std.mem.doNotOptimizeAway(jsalloc.WasmFree);
    std.mem.doNotOptimizeAway(jsalloc.WasmFreeAll);
    std.mem.doNotOptimizeAway(jsalloc.WasmListAllocs);
}

export fn FindPath(grid_str: [*c]const u8, size: usize, pathfinding_type: usize) void {
    std.debug.assert(grid_str != 0);
    const grid_slice = grid_str[0..size];
    std.log.debug("{s}", .{grid_slice});
    find_path(grid_slice, @enumFromInt(pathfinding_type)) catch |e| WasmError(e);
    FlushPrint();
}

fn find_path(grid_slice: []const u8, pathfinding_type: nodes.NodeMap.PathfindingType) !void {
    var nmap = try nodes.NodeMap.init(allocator, grid_slice);
    defer nmap.deinit(allocator);
    switch (pathfinding_type) {
        .minimum_cost_node => try nmap.mcn_path(allocator),
        .shortest_steps_node => try nmap.ssn_path(allocator),
        .brute_forcing => try nmap.brute_force_path(allocator),
        .mst_and_traversal => try nmap.mst_and_traversal(allocator),
    }
}
