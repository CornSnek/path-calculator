const std = @import("std");
pub const Node = packed struct {
    visited: bool,
    cost: u31,
    pub fn format(self: Node, comptime _: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        try std.fmt.formatInt(self.cost, 10, .lower, options, writer);
        if (self.visited)
            try writer.writeByte('v');
    }
};
pub const Coordinate = struct {
    x: u32,
    y: u32,
    pub fn format(self: Coordinate, comptime _: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.writeAll("C[");
        try std.fmt.formatInt(self.x, 10, .lower, options, writer);
        try writer.writeByte(',');
        try std.fmt.formatInt(self.y, 10, .lower, options, writer);
        try writer.writeByte(']');
    }
    pub fn block_sort_fn(_: void, lhs: Coordinate, rhs: Coordinate) bool {
        if (lhs.x != rhs.x) {
            return lhs.x < rhs.x;
        } else return lhs.y < rhs.y;
    }
    pub fn binary_search_fn(_: void, key: Coordinate, mid: Coordinate) std.math.Order {
        if (key.x != mid.x) {
            return std.math.order(key.x, mid.x);
        } else return std.math.order(key.y, mid.y);
    }
};
/// To get a copy of NodeMap iterations of each coordinate.
/// Each repeated node visit then reduces the cost to NodeMap.revisit_cost.
pub const VisitedMap = struct {
    map: std.ArrayListUnmanaged(bool) = .{},
    pub fn init(allocator: std.mem.Allocator, nmap: NodeMap) !VisitedMap {
        var self: VisitedMap = .{ .map = .{} };
        errdefer self.map.deinit(allocator);
        for (nmap.map.items) |node|
            try self.map.append(allocator, node.visited);
        return self;
    }
    fn clone(self: VisitedMap, allocator: std.mem.Allocator) !VisitedMap {
        return .{ .map = try self.map.clone(allocator) };
    }
    fn mark(self: *VisitedMap, nmap: NodeMap, x: u32, y: u32, comptime safety: bool) void {
        if (safety) {
            std.debug.assert(x < nmap.width);
            std.debug.assert(y < nmap.height);
        }
        self.map.items[y * nmap.width + x] = true;
    }
    fn coord(self: VisitedMap, nmap: NodeMap, x: u32, y: u32, comptime safety: bool) if (safety) ?bool else bool {
        if (safety) {
            if (x >= nmap.width) return null;
            if (y >= nmap.height) return null;
        }
        return self.map.items[y * nmap.width + x];
    }
    pub fn deinit(self: *VisitedMap, allocator: std.mem.Allocator) void {
        self.map.deinit(allocator);
    }
};
pub const NodeMap = struct {
    ///Already visiting a node becomes this value.
    pub const DefaultRevisitCost: u31 = 5;
    width: u32,
    height: u32,
    start_x: u32,
    start_y: u32,
    revisit_cost: u31 = DefaultRevisitCost,
    map: std.ArrayListUnmanaged(Node),
    coordinates: std.ArrayListUnmanaged(Coordinate),
    pub fn init(allocator: std.mem.Allocator, str_map: []const u8) !NodeMap {
        const ReadState = enum { RevisitOrMap, Map, GotoCoordinates };
        var read_state: ReadState = .RevisitOrMap;
        var nmap: NodeMap = .{
            .width = 0,
            .height = 0,
            .start_x = undefined,
            .start_y = undefined,
            .map = .{},
            .coordinates = .{},
        };
        var start_point_set: bool = false;
        errdefer nmap.deinit(allocator);
        var line_it = std.mem.tokenizeScalar(u8, str_map, '\n');
        while (line_it.next()) |line| {
            //std.log.debug("{s}\n", .{line});
            var num_it = std.mem.tokenizeScalar(u8, line, ' ');
            if (read_state != .GotoCoordinates) {
                if (std.mem.eql(u8, line, "c")) {
                    read_state = .GotoCoordinates;
                } else {
                    if (read_state == .RevisitOrMap) { //Check if only the first line has "r".
                        if (line[0] == 'r') {
                            nmap.revisit_cost = try std.fmt.parseInt(u31, line[1..], 10);
                            read_state = .Map;
                            continue;
                        }
                    }
                    var count_width: u32 = 0;
                    while (num_it.next()) |num| {
                        if (std.mem.eql(u8, num, "s")) {
                            if (start_point_set) return error.MoreThanOneStartPointSet;
                            nmap.start_x = count_width;
                            nmap.start_y = nmap.height;
                            start_point_set = true;
                            try nmap.map.append(allocator, .{ .visited = true, .cost = nmap.revisit_cost });
                            count_width += 1;
                        } else {
                            var node: Node = undefined;
                            if (std.mem.endsWith(u8, num, "v")) {
                                node = .{ .visited = true, .cost = try std.fmt.parseInt(u31, num[0 .. num.len - 1], 10) };
                            } else {
                                node = .{ .visited = false, .cost = try std.fmt.parseInt(u31, num, 10) };
                            }
                            try nmap.map.append(allocator, node);
                            count_width += 1;
                        }
                    }
                    if (nmap.width != 0) {
                        if (nmap.width != count_width) {
                            return error.UnevenWidth;
                        }
                    } else {
                        nmap.width = count_width;
                    }
                    nmap.height += 1;
                }
            } else {
                var coords_it = std.mem.tokenizeScalar(u8, line, ' ');
                while (coords_it.next()) |slice| {
                    const comma_i = std.mem.indexOf(u8, slice, ",") orelse return error.CommaRequiredForCoordinate;
                    const coord_x = try std.fmt.parseInt(u32, slice[0..comma_i], 10);
                    const coord_y = try std.fmt.parseInt(u32, slice[comma_i + 1 ..], 10);
                    if (coord_x >= nmap.width or coord_y >= nmap.height) return error.CoordinateOutOfBounds;
                    try nmap.coordinates.append(allocator, .{ .x = coord_x, .y = coord_y });
                }
            }
        }
        if (!start_point_set)
            return error.NoStartPointSet;
        if (nmap.width == 0 and nmap.height == 0)
            return error.Empty;
        if (nmap.start_x >= nmap.width or nmap.start_y >= nmap.height)
            return error.StartOutOfBounds;
        //std.log.debug("{any}\n", .{nmap});
        return nmap;
    }
    fn coord(self: NodeMap, x: u32, y: u32, comptime safety: bool) if (safety) ?Node else Node {
        if (safety) {
            if (x >= self.width) return null;
            if (y >= self.height) return null;
        }
        return self.map.items[y * self.width + x];
    }
    pub const Direction = enum { none, left, right, up, down };
    /// .lastc := the last coordinate that edited for the least cost, null if root/start node
    const DijkstraStruct = struct {
        thisc: Coordinate,
        lastc: ?Coordinate = null,
        lastd: Direction = .none,
        disabled: bool = false,
        cost: u32,
    };
    const DijkstraStructMap = std.ArrayListUnmanaged(DijkstraStruct);
    /// Priority queue where the highest priority (as lowest cost) is popped first.
    const MapPQ = struct {
        p: std.ArrayListUnmanaged(u32) = .{},
        map: std.ArrayListUnmanaged(std.ArrayListUnmanaged(Coordinate)) = .{},
        pub fn format(self: MapPQ, comptime _: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
            for (0..self.p.items.len) |i| {
                try writer.writeByte('[');
                try std.fmt.formatInt(self.p.items[i], 10, .lower, options, writer);
                try writer.writeAll("=>");
                try std.fmt.format(writer, "{any}", .{self.map.items[i].items});
                try writer.writeAll("],");
            }
        }
        ///Binary search an existing priority to append to its list, otherwise add it.
        fn add(self: *MapPQ, allocator: std.mem.Allocator, c: Coordinate, p: u32) !void {
            if (self.map.items.len != 0) {
                var low_i: usize = 0;
                var high_i: usize = self.p.items.len - 1;
                var mid_i = high_i / 2;
                while (high_i >= low_i) : (mid_i = (high_i + low_i) / 2) {
                    //std.log.debug("self.p.items[mid_i] {}, p {} mid_i {}\n", .{ self.p.items[mid_i], p, mid_i });
                    if (p == self.p.items[mid_i]) {
                        try self.map.items[mid_i].append(allocator, c);
                        return;
                    } else if (p > self.p.items[mid_i]) {
                        if (mid_i != 0) {
                            high_i = mid_i - 1;
                        } else break;
                    } else {
                        low_i = mid_i + 1;
                    }
                } //The priority added may be bigger or smaller than at mid_i priority.
                const is_smaller: u1 = @intFromBool(p < self.p.items[mid_i]);
                try self.p.insert(allocator, mid_i + is_smaller, p);
                try self.map.insert(allocator, mid_i + is_smaller, .{});
                try self.map.items[mid_i + is_smaller].append(allocator, c);
            } else {
                try self.p.append(allocator, p);
                try self.map.append(allocator, .{});
                try self.map.items[0].append(allocator, c);
            }
        }
        const UpdateEnum = enum { Success, PriorityNotFound, CoordinateNotFound, Empty };
        fn update(self: *MapPQ, allocator: std.mem.Allocator, c: Coordinate, old_p: u32, new_p: u32) !UpdateEnum {
            if (self.p.items.len == 0) return .Empty;
            var low_i: usize = 0;
            var high_i: usize = self.p.items.len - 1;
            var mid_i = high_i / 2;
            while (high_i >= low_i) : (mid_i = (high_i + low_i) / 2) {
                if (old_p == self.p.items[mid_i]) {
                    for (self.map.items[mid_i].items, 0..) |old_c, coord_i| {
                        if (c.x == old_c.x and c.y == old_c.y) {
                            _ = self.map.items[mid_i].orderedRemove(coord_i);
                            if (self.map.items[mid_i].items.len == 0) {
                                var pq_map = self.map.orderedRemove(mid_i);
                                pq_map.deinit(allocator);
                                _ = self.p.orderedRemove(mid_i);
                            }
                            try self.add(allocator, c, new_p);
                            return .Success;
                        }
                    } else return .CoordinateNotFound;
                } else if (old_p > self.p.items[mid_i]) {
                    if (mid_i != 0) {
                        high_i = mid_i - 1;
                    } else return .PriorityNotFound;
                } else {
                    low_i = mid_i + 1;
                }
            }
            return .PriorityNotFound;
        }
        /// The map removes an element in a list by 0th index (Time complexity is n)
        fn pop(self: *MapPQ, allocator: std.mem.Allocator) ?Coordinate {
            if (self.p.items.len == 0) return null;
            const c = (&self.map.items[self.map.items.len - 1]).orderedRemove(0);
            //std.log.debug("Last cost: {}\n", .{self.p.getLast()});
            if (self.map.getLast().items.len == 0) {
                var pq_map = self.map.pop();
                pq_map.deinit(allocator);
                _ = self.p.pop();
            }
            return c;
        }
        fn deinit(self: *MapPQ, allocator: std.mem.Allocator) void {
            self.p.deinit(allocator);
            for (self.map.items) |*pq_map| {
                pq_map.deinit(allocator);
            }
            self.map.deinit(allocator);
        }
    };
    const NodeResult = struct {
        coord: Coordinate,
        cost: u32,
        direction: Direction = .none,
        pub fn format(self: NodeResult, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
            try writer.writeByte('[');
            try Coordinate.format(self.coord, fmt, options, writer);
            try writer.writeByte(' ');
            if (self.direction != .none) {
                try writer.writeAll("(d: ");
                try writer.print("{s}", .{@tagName(self.direction)});
                try writer.writeAll(") ");
            }
            try writer.writeAll("(c: ");
            try std.fmt.formatInt(self.cost, 10, .lower, options, writer);
            try writer.writeAll(")]");
        }
    };
    const PathResult = struct {
        cost: u32 = std.math.maxInt(u32),
        path: std.ArrayListUnmanaged(NodeResult) = .{},
        fn clone(self: PathResult, allocator: std.mem.Allocator) !PathResult {
            return .{ .cost = self.cost, .path = try self.path.clone(allocator) };
        }
        fn deinit(self: *PathResult, allocator: std.mem.Allocator) void {
            self.path.deinit(allocator);
        }
    };
    ///If no path is found, only cend is the path with the total cost of std.math.maxInt(u32)
    pub fn dijkstra(
        self: NodeMap,
        allocator: std.mem.Allocator,
        vmap: *VisitedMap,
        cstart: Coordinate,
        cend: Coordinate,
        exclude: ?[]const Coordinate,
    ) !PathResult {
        //std.log.debug("Dijkstra's from {} to {}, exclude {any}\n", .{ cstart, cend, exclude });
        var dijkstra_struct_map: DijkstraStructMap = .{};
        defer dijkstra_struct_map.deinit(allocator);
        const start_i = cstart.y * self.width + cstart.x;
        for (0..self.height) |y| {
            for (0..self.width) |x| {
                if (start_i != y * self.width + x) {
                    try dijkstra_struct_map.append(allocator, .{
                        .thisc = .{ .x = @intCast(x), .y = @intCast(y) },
                        .cost = std.math.maxInt(u32),
                    });
                } else {
                    try dijkstra_struct_map.append(allocator, .{
                        .thisc = .{ .x = @intCast(x), .y = @intCast(y) },
                        .cost = 0,
                    });
                }
            }
        }
        if (exclude) |exc| { //Any coordinate not cend is excluded from being a node path.
            for (exc) |coord_exc| {
                if (coord_exc.x != cend.x or coord_exc.y != cend.y)
                    dijkstra_struct_map.items[coord_exc.y * self.width + coord_exc.x].disabled = true;
            }
        }
        var map_pq: MapPQ = .{};
        defer map_pq.deinit(allocator);
        try map_pq.add(allocator, .{ .x = cstart.x, .y = cstart.y }, 0);
        //std.log.debug("{} {any}\n", .{ cstart, dijkstra_struct_map.items });
        while (map_pq.pop(allocator)) |this_coord| {
            var left_coord: ?Coordinate = null;
            if (this_coord.x != 0) {
                left_coord = .{ .x = this_coord.x - 1, .y = this_coord.y };
                if (dijkstra_struct_map.items[left_coord.?.y * self.width + left_coord.?.x].disabled) left_coord = null;
            }
            var right_coord: ?Coordinate = null;
            if (this_coord.x != self.width - 1) {
                right_coord = .{ .x = this_coord.x + 1, .y = this_coord.y };
                if (dijkstra_struct_map.items[right_coord.?.y * self.width + right_coord.?.x].disabled) right_coord = null;
            }
            var lower_coord: ?Coordinate = null;
            if (this_coord.y != self.height - 1) {
                lower_coord = .{ .x = this_coord.x, .y = this_coord.y + 1 };
                if (dijkstra_struct_map.items[lower_coord.?.y * self.width + lower_coord.?.x].disabled) lower_coord = null;
            }
            var upper_coord: ?Coordinate = null;
            if (this_coord.y != 0) {
                upper_coord = .{ .x = this_coord.x, .y = this_coord.y - 1 };
                if (dijkstra_struct_map.items[upper_coord.?.y * self.width + upper_coord.?.x].disabled) upper_coord = null;
            }
            //std.log.debug("t: {any} l: {any} r: {any} d: {any} u: {any}\n", .{ this_coord, left_coord, right_coord, lower_coord, upper_coord });
            const tile_cost = dijkstra_struct_map.items[this_coord.y * self.width + this_coord.x].cost;
            std.debug.assert(tile_cost != std.math.maxInt(u32));
            if (left_coord != null) {
                const left_tile_cost = dijkstra_struct_map.items[left_coord.?.y * self.width + left_coord.?.x].cost;
                const left_node = self.map.items[left_coord.?.y * self.width + left_coord.?.x];
                const left_vmap: bool = vmap.coord(self, left_coord.?.x, left_coord.?.y, false);
                const left_new_cost: u32 = if (left_node.visited or left_vmap) self.revisit_cost else left_node.cost;
                if (left_new_cost + tile_cost < left_tile_cost) {
                    if (try map_pq.update(allocator, left_coord.?, dijkstra_struct_map.items[left_coord.?.y * self.width + left_coord.?.x].cost, left_new_cost) != .Success)
                        try map_pq.add(allocator, left_coord.?, left_new_cost);
                    const left_ptr = &dijkstra_struct_map.items[left_coord.?.y * self.width + left_coord.?.x];
                    left_ptr.cost = left_new_cost;
                    left_ptr.lastc = this_coord;
                    left_ptr.lastd = .left;
                }
            }
            if (right_coord != null) {
                const right_tile_cost = dijkstra_struct_map.items[right_coord.?.y * self.width + right_coord.?.x].cost;
                const right_node = self.map.items[right_coord.?.y * self.width + right_coord.?.x];
                const right_vmap: bool = vmap.coord(self, right_coord.?.x, right_coord.?.y, false);
                const right_new_cost: u32 = if (right_node.visited or right_vmap) self.revisit_cost else right_node.cost;
                if (right_new_cost + tile_cost < right_tile_cost) {
                    if (try map_pq.update(allocator, right_coord.?, dijkstra_struct_map.items[right_coord.?.y * self.width + right_coord.?.x].cost, right_new_cost) != .Success)
                        try map_pq.add(allocator, right_coord.?, right_new_cost);
                    const right_ptr = &dijkstra_struct_map.items[right_coord.?.y * self.width + right_coord.?.x];
                    right_ptr.cost = right_new_cost;
                    right_ptr.lastc = this_coord;
                    right_ptr.lastd = .right;
                }
            }
            if (lower_coord != null) {
                const lower_tile_cost = dijkstra_struct_map.items[lower_coord.?.y * self.width + lower_coord.?.x].cost;
                const lower_node = self.map.items[lower_coord.?.y * self.width + lower_coord.?.x];
                const lower_vmap: bool = vmap.coord(self, lower_coord.?.x, lower_coord.?.y, false);
                const lower_new_cost: u32 = if (lower_node.visited or lower_vmap) self.revisit_cost else lower_node.cost;
                if (lower_new_cost + tile_cost < lower_tile_cost) {
                    if (try map_pq.update(allocator, lower_coord.?, dijkstra_struct_map.items[lower_coord.?.y * self.width + lower_coord.?.x].cost, lower_new_cost) != .Success)
                        try map_pq.add(allocator, lower_coord.?, lower_new_cost);
                    const lower_ptr = &dijkstra_struct_map.items[lower_coord.?.y * self.width + lower_coord.?.x];
                    lower_ptr.cost = lower_new_cost;
                    lower_ptr.lastc = this_coord;
                    lower_ptr.lastd = .down;
                }
            }
            if (upper_coord != null) {
                const upper_tile_cost = dijkstra_struct_map.items[upper_coord.?.y * self.width + upper_coord.?.x].cost;
                const upper_node = self.map.items[upper_coord.?.y * self.width + upper_coord.?.x];
                const upper_vmap: bool = vmap.coord(self, upper_coord.?.x, upper_coord.?.y, false);
                const upper_new_cost: u32 = if (upper_node.visited or upper_vmap) self.revisit_cost else upper_node.cost;
                if (upper_new_cost + tile_cost < upper_tile_cost) {
                    if (try map_pq.update(allocator, upper_coord.?, dijkstra_struct_map.items[upper_coord.?.y * self.width + upper_coord.?.x].cost, upper_new_cost) != .Success)
                        try map_pq.add(allocator, upper_coord.?, upper_new_cost);
                    const upper_ptr = &dijkstra_struct_map.items[upper_coord.?.y * self.width + upper_coord.?.x];
                    upper_ptr.cost = upper_new_cost;
                    upper_ptr.lastc = this_coord;
                    upper_ptr.lastd = .up;
                }
            }
        }
        var optimal_path: std.ArrayListUnmanaged(NodeResult) = .{};
        errdefer optimal_path.deinit(allocator);
        const cend_node = self.map.items[cend.y * self.width + cend.x];
        const cend_cost = if (cend_node.visited) self.revisit_cost else cend_node.cost;
        try optimal_path.append(allocator, .{ .coord = cend, .cost = cend_cost, .direction = dijkstra_struct_map.items[cend.y * self.width + cend.x].lastd });
        var total_cost = dijkstra_struct_map.items[cend.y * self.width + cend.x].cost;
        var current_c: ?Coordinate = cend;
        while (current_c != null) {
            if (dijkstra_struct_map.items[current_c.?.y * self.width + current_c.?.x].lastc) |exists_c| {
                const dijk_node = dijkstra_struct_map.items[exists_c.y * self.width + exists_c.x];
                total_cost += dijk_node.cost;
                try optimal_path.append(allocator, .{ .coord = exists_c, .cost = dijk_node.cost, .direction = dijk_node.lastd });
                current_c = exists_c;
            } else current_c = null;
        }
        for (optimal_path.items) |c| vmap.mark(self, c.coord.x, c.coord.y, false);
        for (0..optimal_path.items.len / 2) |i| { //Reverse to get path
            std.mem.swap(NodeResult, &optimal_path.items[i], &optimal_path.items[optimal_path.items.len - 1 - i]);
        }
        //std.log.debug("cost: {} path: {any}\n", .{ total_cost, optimal_path.items });
        return .{ .cost = total_cost, .path = optimal_path };
    }
    fn u32_sort(_: void, lhs: u32, rhs: u32) bool {
        return lhs < rhs;
    }
    pub fn u32_bin_search(_: void, key: u32, mid: u32) std.math.Order {
        return std.math.order(key, mid);
    }
    const FrequencyCount = struct { value: u32, count: u32 };
    fn average(self: NodeMap, vmap: VisitedMap) f32 {
        const total_nodes: u32 = self.width * self.height;
        var sum: u31 = 0;
        for (0..self.height) |y| {
            for (0..self.width) |x| {
                sum += if (!vmap.coord(self, @intCast(x), @intCast(y), false)) self.map.items[y * self.width + x].cost else self.revisit_cost;
            }
        }
        return @as(f32, @floatFromInt(sum)) / @as(f32, @floatFromInt(total_nodes));
    }
    pub const PathfindingType = enum {
        minimum_cost_node,
        shortest_steps_node,
        brute_forcing,
        pub fn alt_names(self: PathfindingType) []const u8 {
            return switch (self) {
                .minimum_cost_node => "Minimum Cost to Next Node",
                .shortest_steps_node => "Shortest Steps to Next Node",
                .brute_forcing => "Brute Forcing",
            };
        }
        pub fn description(self: PathfindingType) []const u8 {
            return switch (self) {
                .minimum_cost_node => "The least cost node is chosen first for each path.",
                .shortest_steps_node => "The node with the shortest amount of steps is chosen first for each path.",
                .brute_forcing => "Warning! Brute Forcing is extremely slow when selecting a large amount of coordinates, but this pathfinding guarantees the lowest cost.",
            };
        }
    };
    //Pathfinder byte format: {Combined Path Total Cost, Total number of X paths, X1..., X2... }
    //X format: {Bytes to read (Excluding this byte), Start x, Start y, End x, End y, Path Total Cost, Number of D Directions, D1..., D2... }
    //D format: {Cost of direction, nodes.NodeMap.Direction enum}
    pub var PathfinderArrayList: std.ArrayListUnmanaged(u32) = .{};
    extern fn ParsePathfinder([*c]u32, usize, bool) void;
    fn wasm_start_pathfinder(allocator: std.mem.Allocator) !void {
        if (@import("builtin").os.tag == .freestanding) {
            PathfinderArrayList.clearRetainingCapacity();
            try PathfinderArrayList.appendSlice(allocator, &.{ 0, 0 }); //&.{Total cost of nodes, Total paths}
        }
    }
    fn wasm_append_path_bytes(shortest_path: PathResult, allocator: std.mem.Allocator) !void {
        if (@import("builtin").os.tag == .freestanding) {
            var path_bytes = std.ArrayList(u32).init(allocator);
            defer path_bytes.deinit();
            try path_bytes.append(6); //Total number of bytes to read for this path (6 for the bytes appended below excluding the for loop)
            try path_bytes.append(shortest_path.path.items[0].coord.x);
            try path_bytes.append(shortest_path.path.items[0].coord.y);
            try path_bytes.append(shortest_path.path.getLast().coord.x);
            try path_bytes.append(shortest_path.path.getLast().coord.y);
            try path_bytes.append(shortest_path.cost);
            try path_bytes.append(@as(u32, @intCast(shortest_path.path.items.len)) - 1);
            for (shortest_path.path.items) |pr| {
                if (pr.direction != .none) {
                    try path_bytes.append(pr.cost);
                    try path_bytes.append(@intFromEnum(pr.direction));
                    path_bytes.items[0] += 2;
                }
            }
            std.log.debug("{any}\n", .{path_bytes.items});
            PathfinderArrayList.items[0] += shortest_path.cost;
            PathfinderArrayList.items[1] += 1;
            try PathfinderArrayList.appendSlice(allocator, path_bytes.items);
        }
    }
    fn wasm_end_pathfinder(continue_disable: bool) void {
        if (@import("builtin").os.tag == .freestanding) ParsePathfinder(PathfinderArrayList.items.ptr, PathfinderArrayList.items.len, continue_disable);
    }
    ///Minimum Cost to node pathfinder
    pub fn mcn_path(self: NodeMap, allocator: std.mem.Allocator) !void {
        try wasm_start_pathfinder(allocator);
        var current_c: Coordinate = .{ .x = self.start_x, .y = self.start_y };
        var coordinates_left = try self.coordinates.clone(allocator);
        defer coordinates_left.deinit(allocator);
        std.sort.block(Coordinate, coordinates_left.items, {}, Coordinate.block_sort_fn);
        var shortest_vmap: VisitedMap = try VisitedMap.init(allocator, self);
        defer shortest_vmap.deinit(allocator);
        var total_cost: u32 = 0;
        while (coordinates_left.items.len != 0) {
            var lowest_cost: u32 = std.math.maxInt(u32);
            var shortest_path: PathResult = .{};
            defer shortest_path.deinit(allocator);
            var temp_vmap = try shortest_vmap.clone(allocator);
            defer temp_vmap.deinit(allocator);
            var possible_next_c: ?Coordinate = null;
            for (coordinates_left.items) |next_c| {
                var possible_shortest_vmap = try temp_vmap.clone(allocator);
                errdefer possible_shortest_vmap.deinit(allocator);
                var result = try self.dijkstra(allocator, &possible_shortest_vmap, current_c, next_c, coordinates_left.items);
                if (result.path.items.len == 1) { //Exclude unreachable nodes.
                    possible_shortest_vmap.deinit(allocator);
                    result.deinit(allocator);
                    continue;
                }
                if (result.cost < lowest_cost) {
                    lowest_cost = result.cost;
                    shortest_path.deinit(allocator);
                    shortest_path = result;
                    shortest_vmap.deinit(allocator);
                    shortest_vmap = possible_shortest_vmap;
                    possible_next_c = next_c;
                } else {
                    possible_shortest_vmap.deinit(allocator);
                    result.deinit(allocator);
                }
                //std.log.debug("{d} {} {any}\n", .{ avg, next_c, result.path.items });
            }
            total_cost += shortest_path.cost;
            current_c = possible_next_c.?;
            std.log.debug("{} is the chosen coordinate with path {any} (Total cost is {})\n", .{ current_c, shortest_path.path.items, shortest_path.cost });
            try wasm_append_path_bytes(shortest_path, allocator);
            const remove_c = std.sort.binarySearch(Coordinate, current_c, coordinates_left.items, {}, Coordinate.binary_search_fn).?;
            _ = coordinates_left.orderedRemove(remove_c);
            for (shortest_path.path.items) |add_visit_c|
                shortest_vmap.mark(self, add_visit_c.coord.x, add_visit_c.coord.y, false);
        }
        std.log.debug("Total cost for this pathfind is {}.\n", .{total_cost});
        wasm_end_pathfinder(false);
    }
    const StepsCost = struct {
        steps: usize,
        cost: u32,
        fn max() StepsCost {
            return .{ .steps = std.math.maxInt(usize), .cost = std.math.maxInt(u32) };
        }
        fn order(lhs: StepsCost, rhs: StepsCost) std.math.Order {
            if (lhs.steps != rhs.steps) {
                return std.math.order(lhs.steps, rhs.steps);
            } else return std.math.order(lhs.cost, rhs.cost);
        }
    };
    ///Shortest Steps to node pathfinder
    pub fn ssn_path(self: NodeMap, allocator: std.mem.Allocator) !void {
        try wasm_start_pathfinder(allocator);
        var current_c: Coordinate = .{ .x = self.start_x, .y = self.start_y };
        var coordinates_left = try self.coordinates.clone(allocator);
        defer coordinates_left.deinit(allocator);
        std.sort.block(Coordinate, coordinates_left.items, {}, Coordinate.block_sort_fn);
        var shortest_vmap: VisitedMap = try VisitedMap.init(allocator, self);
        defer shortest_vmap.deinit(allocator);
        var total_cost: u32 = 0;
        while (coordinates_left.items.len != 0) {
            var lowest_cost: StepsCost = StepsCost.max();
            var shortest_path: PathResult = .{};
            defer shortest_path.deinit(allocator);
            var temp_vmap = try shortest_vmap.clone(allocator);
            defer temp_vmap.deinit(allocator);
            var possible_next_c: ?Coordinate = null;
            for (coordinates_left.items) |next_c| {
                var possible_shortest_vmap = try temp_vmap.clone(allocator);
                errdefer possible_shortest_vmap.deinit(allocator);
                var result = try self.dijkstra(allocator, &possible_shortest_vmap, current_c, next_c, coordinates_left.items);
                if (result.path.items.len == 1) { //Exclude unreachable nodes.
                    possible_shortest_vmap.deinit(allocator);
                    result.deinit(allocator);
                    continue;
                }
                const result_cost: StepsCost = .{ .steps = result.path.items.len - 1, .cost = result.cost };
                if (StepsCost.order(result_cost, lowest_cost) == .lt) {
                    lowest_cost = result_cost;
                    shortest_path.deinit(allocator);
                    shortest_path = result;
                    shortest_vmap.deinit(allocator);
                    shortest_vmap = possible_shortest_vmap;
                    possible_next_c = next_c;
                } else {
                    possible_shortest_vmap.deinit(allocator);
                    result.deinit(allocator);
                }
            }
            total_cost += shortest_path.cost;
            current_c = possible_next_c.?;
            std.log.debug("{} is the chosen coordinate with path {any} (Total cost is {})\n", .{ current_c, shortest_path.path.items, shortest_path.cost });
            try wasm_append_path_bytes(shortest_path, allocator);
            const remove_c = std.sort.binarySearch(Coordinate, current_c, coordinates_left.items, {}, Coordinate.binary_search_fn).?;
            _ = coordinates_left.orderedRemove(remove_c);
            for (shortest_path.path.items) |add_visit_c|
                shortest_vmap.mark(self, add_visit_c.coord.x, add_visit_c.coord.y, false);
        }
        std.log.debug("Total cost for this pathfind is {}.\n", .{total_cost});
        wasm_end_pathfinder(false);
    }
    /// Next lexicographical order of a permutation of a slice.
    fn permutation_next(comptime T: type, items: []T, context: anytype, comptime less_than: fn (@TypeOf(context), lhs: T, rhs: T) bool) bool {
        for (0..items.len - 1) |i| {
            const pivot_i = items.len - 2 - i;
            if (less_than(context, items[pivot_i], items[pivot_i + 1])) {
                for (0..items.len - 1 - pivot_i) |j| {
                    const righmost_larger_i = items.len - 1 - j;
                    if (less_than(context, items[pivot_i], items[righmost_larger_i])) {
                        std.mem.swap(T, &items[pivot_i], &items[righmost_larger_i]);
                        std.mem.reverse(T, items[pivot_i + 1 ..]);
                        return true;
                    }
                }
            }
        }
        return false;
    }
    /// To cache consecutive coordinates/paths/visitem maps together when choosing a new permutation of coordinates.
    const BFResults = struct {
        const BFNode = struct { vmap: VisitedMap, coord: Coordinate, path: PathResult };
        nodes: std.ArrayListUnmanaged(BFNode) = .{},
        fn get_total_cost(self: BFResults) u32 {
            var res: u32 = 0;
            for (self.nodes.items) |bfn| res += bfn.path.cost;
            return res;
        }
        fn last_vmap(self: BFResults, allocator: std.mem.Allocator) !?VisitedMap {
            return if (self.nodes.items.len != 0) try self.nodes.getLast().vmap.clone(allocator) else null;
        }
        fn last_coord(self: BFResults) ?Coordinate {
            return if (self.nodes.items.len != 0) self.nodes.getLast().coord else null;
        }
        fn path_results(self: BFResults, allocator: std.mem.Allocator) !std.ArrayListUnmanaged(PathResult) {
            var list: std.ArrayListUnmanaged(PathResult) = .{};
            errdefer list.deinit(allocator);
            try list.ensureTotalCapacity(allocator, self.nodes.items.len);
            for (self.nodes.items) |bfn| list.appendAssumeCapacity(try bfn.path.clone(allocator));
            return list;
        }
        fn remove_non_consecutive(self: *BFResults, coords: []const Coordinate, allocator: std.mem.Allocator) usize {
            var consecutive_num: usize = 0;
            for (0..self.nodes.items.len) |i| {
                const bfcoord = self.nodes.items[i].coord;
                if (coords[i].x == bfcoord.x and coords[i].y == bfcoord.y) {
                    consecutive_num += 1;
                } else break;
            }
            for (consecutive_num..self.nodes.items.len) |i| { //Deinitialize and remove non-consecutive nodes.
                const bfn = &self.nodes.items[i];
                bfn.vmap.deinit(allocator);
                bfn.path.deinit(allocator);
            }
            self.nodes.items.len = consecutive_num;
            return consecutive_num;
        }
        fn deinit(self: *BFResults, allocator: std.mem.Allocator) void {
            for (self.nodes.items) |*bfn| {
                bfn.vmap.deinit(allocator);
                bfn.path.deinit(allocator);
            }
            self.nodes.deinit(allocator);
        }
    };
    extern fn CalculateBruteForceEarly() bool;
    extern fn GetOutput() bool;
    extern fn GetCancel() bool;
    extern fn OutputBruteForcing([*c]const u8, usize) void;
    pub fn brute_force_path(self: NodeMap, allocator: std.mem.Allocator) !void {
        var sorted_coordinates = try self.coordinates.clone(allocator);
        defer sorted_coordinates.deinit(allocator);
        std.sort.block(Coordinate, sorted_coordinates.items, {}, Coordinate.block_sort_fn);
        var lowest_results: std.ArrayListUnmanaged(PathResult) = .{};
        defer {
            for (lowest_results.items) |*res| {
                res.deinit(allocator);
            }
            lowest_results.deinit(allocator);
        }
        var lowest_total_cost: u32 = std.math.maxInt(u32);
        var bfr: BFResults = .{};
        defer bfr.deinit(allocator);
        var test_output: std.ArrayListUnmanaged(u8) = .{};
        defer test_output.deinit(allocator);
        while (!GetCancel()) {
            var next_coordinates = try sorted_coordinates.clone(allocator);
            defer next_coordinates.deinit(allocator);
            if (@import("builtin").os.tag == .freestanding) {
                if (GetOutput()) {
                    for (next_coordinates.items) |c| {
                        try test_output.writer(allocator).print("[{: >2},{: >2}] => ", .{ c.x, c.y });
                    }
                    test_output.items.len -= " => ".len;
                    try test_output.writer(allocator).print("<br>Current Lowest Total Cost: {}. You can press 'Get Brute Force Path Early' to get a calculated path, but the lowest cost path may not be found yet.", .{lowest_total_cost});
                    OutputBruteForcing(test_output.items.ptr, test_output.items.len);
                    test_output.items.len = 0;
                }
            }
            const consecutive_num = bfr.remove_non_consecutive(next_coordinates.items, allocator);
            var possible_lowest_cost: u32 = bfr.get_total_cost();
            var possible_lowest_results: std.ArrayListUnmanaged(PathResult) = try bfr.path_results(allocator);
            var vmap: VisitedMap = try bfr.last_vmap(allocator) orelse try VisitedMap.init(allocator, self);
            defer vmap.deinit(allocator);
            var current_c: Coordinate = bfr.last_coord() orelse .{ .x = self.start_x, .y = self.start_y };
            next_permutation: {
                for (consecutive_num..next_coordinates.items.len) |i| {
                    const next_c = next_coordinates.items[i];
                    var result = try self.dijkstra(allocator, &vmap, current_c, next_c, next_coordinates.items[i + 1 ..]);
                    if (result.path.items.len == 1) { //Exclude paths with unreachable nodes.
                        result.deinit(allocator);
                        for (possible_lowest_results.items) |*res| {
                            res.deinit(allocator);
                        }
                        possible_lowest_results.deinit(allocator);
                        break :next_permutation;
                    }
                    try possible_lowest_results.append(allocator, result);
                    possible_lowest_cost += result.cost;
                    if (possible_lowest_cost >= lowest_total_cost) {
                        for (possible_lowest_results.items) |*res| {
                            res.deinit(allocator);
                        }
                        possible_lowest_results.deinit(allocator);
                        break :next_permutation;
                    }
                    try bfr.nodes.append(allocator, .{ .coord = next_c, .vmap = try vmap.clone(allocator), .path = try result.clone(allocator) });
                    current_c = next_c;
                }
                std.log.debug("Found lower cost of {}\n", .{possible_lowest_cost});
                for (possible_lowest_results.items) |result| {
                    std.log.debug("{} is the chosen coordinate with path {any} (Total cost is {})\n", .{ result.path.getLast().coord, result.path.items, result.cost });
                }
                std.log.debug("Total cost for this pathfind is {}.\n\n", .{possible_lowest_cost});
                lowest_total_cost = possible_lowest_cost;
                for (lowest_results.items) |*res| {
                    res.deinit(allocator);
                }
                lowest_results.deinit(allocator);
                lowest_results = possible_lowest_results;
            }
            if (@import("builtin").os.tag == .freestanding) {
                if (CalculateBruteForceEarly()) {
                    try wasm_start_pathfinder(allocator);
                    for (lowest_results.items) |result| {
                        try wasm_append_path_bytes(result, allocator);
                    }
                    wasm_end_pathfinder(true);
                }
            }
            if (!permutation_next(Coordinate, sorted_coordinates.items, {}, Coordinate.block_sort_fn)) break;
        }
        if (@import("builtin").os.tag == .freestanding) {
            const bfdesc = PathfindingType.brute_forcing.description();
            OutputBruteForcing(bfdesc.ptr, bfdesc.len);
        }
        try wasm_start_pathfinder(allocator);
        for (lowest_results.items) |result| {
            std.log.debug("{} is the chosen coordinate with path {any} (Total cost is {})\n", .{ result.path.getLast().coord, result.path.items, result.cost });
            try wasm_append_path_bytes(result, allocator);
        }
        wasm_end_pathfinder(false);
        std.log.debug("Total cost for this pathfind is {}.\n", .{lowest_total_cost});
    }
    pub fn deinit(self: *NodeMap, allocator: std.mem.Allocator) void {
        self.map.deinit(allocator);
        self.coordinates.deinit(allocator);
    }
};
