//!This is created because there are problems referencing enums directly in some files.
pub const Direction = @import("nodes.zig").NodeMap.Direction;
pub const PathfindingType = @import("nodes.zig").NodeMap.PathfindingType;
pub const PrintType = enum(i32) { log, warn, err };
