const std = @import("std");
fn EnumsToJSClass(EnumClass: anytype, export_name: []const u8) []const u8 {
    if (@typeInfo(EnumClass) != .Enum) @compileError(@typeName(EnumClass) ++ " must be an enum type.");
    var export_str: []const u8 = &.{};
    export_str = export_str ++ std.fmt.comptimePrint("//Exported Zig enums '{s}' to javascript variable name '{s}'\n", .{ @typeName(EnumClass), export_name });
    export_str = export_str ++ "export class " ++ export_name ++ " {\n";
    const fields = std.meta.fields(EnumClass);
    for (fields) |field|
        export_str = export_str ++ std.fmt.comptimePrint("\tstatic get {s}() {{ return {}; }}\n", .{ field.name, field.value });
    export_str = export_str ++ std.fmt.comptimePrint("\tstatic get $$length() {{ return {}; }}\n", .{fields.len});
    export_str = export_str ++ "\tstatic get $$names() { return Array.from([";
    for (fields) |field|
        export_str = export_str ++ std.fmt.comptimePrint(" \"{s}\",", .{field.name});
    export_str = export_str ++ " ]); }\n";
    for (@typeInfo(EnumClass).Enum.decls) |decl| { //Get string descriptions of enums.
        const DeclType = @TypeOf(@field(EnumClass, decl.name));
        if (@typeInfo(DeclType) == .Fn) {
            const FnInfo = @typeInfo(DeclType).Fn;
            if (FnInfo.return_type == []const u8 and FnInfo.params.len == 1 and FnInfo.params[0].type == EnumClass) {
                export_str = export_str ++ "\tstatic get $" ++ decl.name ++ "() { return Array.from([";
                for (fields) |field|
                    export_str = export_str ++ std.fmt.comptimePrint(" \"{s}\",", .{@field(EnumClass, decl.name)(@enumFromInt(field.value))});
                export_str = export_str ++ " ]); }\n";
            }
        }
    }
    export_str = export_str ++ "};\n\n";
    return export_str;
}
pub fn build(b: *std.Build) !void {
    const www_root = "www";
    const wasm_name = "pathfinder";
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const exe = b.addExecutable(.{
        .name = wasm_name,
        .root_source_file = b.path("src/os_main.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(exe);
    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
    const install_website = b.addInstallDirectory(.{
        .source_dir = b.path(www_root),
        .install_dir = .bin,
        .install_subdir = www_root,
    });

    const install_website_run_step = b.step("website", "Copies website files to bin");
    install_website.step.dependOn(b.getUninstallStep());
    b.getInstallStep().dependOn(&install_website.step);
    install_website_run_step.dependOn(&install_website.step);
    const shared_enums = @import("src/shared_enums.zig");
    const write_export_enums_str = std.fmt.comptimePrint("//This is auto-generated from the build.zig file to use for wasm-javascript reading\n\n{s}{s}{s}", .{
        comptime EnumsToJSClass(shared_enums.Direction, "Direction"),
        comptime EnumsToJSClass(shared_enums.PathfindingType, "PathfindingType"),
        comptime EnumsToJSClass(shared_enums.PrintType, "PrintType"),
    });
    const write_export_enums = b.addWriteFile(
        "wasm_enums_to_js.js",
        write_export_enums_str,
    );
    write_export_enums.step.dependOn(&install_website.step);
    const add_export_enums = b.addInstallDirectory(.{
        .source_dir = write_export_enums.getDirectory(),
        .install_dir = .bin,
        .install_subdir = www_root,
    });
    add_export_enums.step.dependOn(&write_export_enums.step);

    const wasm_exe = b.addExecutable(.{
        .name = wasm_name,
        .root_source_file = b.path("src/wasm_main.zig"),
        .target = b.resolveTargetQuery(.{
            .cpu_arch = .wasm32,
            .os_tag = .freestanding,
        }),
        .optimize = optimize,
    });
    wasm_exe.entry = .disabled;
    wasm_exe.rdynamic = true;
    wasm_exe.root_module.export_symbol_names = &.{
        "FindPath",
        "FlushPrint",
        "PrintBufferMax",
        "WasmListAllocs",
        "WasmAlloc",
        "WasmFree",
        "WasmFreeAll",
    };

    const install_wasm = b.addInstallArtifact(wasm_exe, .{
        .dest_sub_path = try std.fmt.allocPrint(b.allocator, "{s}/{s}.wasm", .{ www_root, wasm_name }),
    });
    const wasm_step = b.step("wasm", "Build wasm binaries and copies files to bin.");
    wasm_step.dependOn(&install_wasm.step);
    install_wasm.step.dependOn(&add_export_enums.step);
    install_wasm.step.dependOn(&install_website.step);

    const run_website_step = b.step("server", "Initializes the wasm step, and runs python http.server");
    const python_http = b.addSystemCommand(&.{ "python", "test_website.py" });
    run_website_step.dependOn(&python_http.step);
    python_http.step.dependOn(&install_wasm.step);
}
