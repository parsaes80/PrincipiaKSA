//! game <game|run> <mods folder, or -> <KSA folder> (<name> <file>)...
//!
//! Installs the mod's files into <mods folder>/PrincipiaKSA, keeping its
//! settings and log, and any file not given (the native library, when only
//! the C# mod is); `run` then starts KSA through StarMap. Without a mods
//! folder: the Borea launcher's active instance, which Borea runs the game
//! from, else KSA's own mods folder.

const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const Allocator = std.mem.Allocator;

pub fn main(init: std.process.Init) !void {
    const arena = init.arena.allocator();
    const io = init.io;
    const args = try init.minimal.args.toSlice(arena);
    if (args.len < 4 or (args.len - 4) % 2 != 0) {
        std.debug.print("usage: game <game|run> <mods folder, or -> <KSA folder> (<name> <file>)...\n", .{});
        return error.Usage;
    }
    const ksa = args[3];
    const home = init.environ_map.get("HOME") orelse init.environ_map.get("USERPROFILE") orelse
        return error.NoHomeFolder;
    const ksa_user = try std.fs.path.join(arena, &.{ home, "Documents", "My Games", "Kitten Space Agency" });
    const borea = if (builtin.os.tag == .macos)
        try std.fs.path.join(arena, &.{ home, "Library", "Application Support", "Borea" })
    else
        try std.fs.path.join(arena, &.{ home, ".local", "share", "Borea" });
    const instance = try boreaInstance(arena, io, borea);

    const mods = if (!std.mem.eql(u8, args[2], "-"))
        args[2]
    else if (instance) |dir|
        try std.fs.path.join(arena, &.{ dir, "mods" })
    else
        try std.fs.path.join(arena, &.{ ksa_user, "mods" });
    const destination = try std.fs.path.join(arena, &.{ mods, "PrincipiaKSA" });
    const cwd = Io.Dir.cwd();
    var i: usize = 4;
    while (i < args.len) : (i += 2) {
        const path = try std.fs.path.join(arena, &.{ destination, args[i] });
        try cwd.createDirPath(io, std.fs.path.dirname(path).?);
        try Io.Dir.copyFile(cwd, args[i + 1], cwd, path, io, .{});
    }
    // The native library this platform loads, given or installed before.
    const native = switch (builtin.os.tag) {
        .windows => "principia_ksa_AVX.dll",
        .macos => "libprincipia_ksa_arm64.dylib",
        else => "libprincipia_ksa_AVX.so",
    };
    const library = try std.fs.path.join(arena, &.{ destination, native });
    cwd.access(io, library, .{}) catch {
        std.debug.print("No native library at {s}: install everything once (zig build game, " ++
            "without -Dcs)\n", .{library});
        return error.NoNativeLibrary;
    };
    std.debug.print("PrincipiaKSA installed to {s} ({d} files)\n", .{ destination, (args.len - 4) / 2 });
    if (!std.mem.eql(u8, args[1], "run")) return;

    // StarMap passes its other arguments on to KSA.
    var argv: std.ArrayList([]const u8) = .empty;
    const host = if (builtin.os.tag == .macos) try std.fs.path.join(arena, &.{ ksa, "KSA" }) else "dotnet";
    try argv.appendSlice(arena, &.{ host, "StarMap.dll" });
    var environ = try init.environ_map.clone(arena);
    var starmap: []const u8 = undefined;
    if (instance) |dir| {
        // As Borea launches it.
        starmap = try boreaSetting(arena, io, borea, "borea-settings.toml", "DirectoryPath") orelse
            try std.fs.path.join(arena, &.{ borea, "Loaders", "StarMap", "0.4.8" });
        try argv.appendSlice(arena, &.{ "-InstancePath", dir });
        try environ.put("STARMAP_INSTANCE_PATH", dir);
    } else {
        starmap = try std.fs.path.join(arena, &.{ ksa_user, "mods", "StarMap" });
    }
    if (builtin.os.tag == .linux) try argv.append(arena, "-wayland");
    std.debug.print("Starting KSA through StarMap in {s}\n", .{starmap});
    var child = try std.process.spawn(io, .{ .argv = argv.items, .cwd = .{ .path = starmap }, .environ_map = &environ });
    const term = try child.wait(io);
    if (!term.success()) return error.GameFailed;
}

// The active instance's folder, if Borea is set up.
fn boreaInstance(arena: Allocator, io: Io, borea: []const u8) !?[]const u8 {
    const id = try boreaSetting(arena, io, borea, "active-instance.toml", "ActiveInstanceId") orelse return null;
    const dir = try std.fs.path.join(arena, &.{ borea, "Instances", id });
    const manifest = try std.fs.path.join(arena, &.{ dir, "manifest.toml" });
    Io.Dir.cwd().access(io, manifest, .{}) catch return null;
    return dir;
}

// `key = "value"` in one of Borea's TOML files.
fn boreaSetting(arena: Allocator, io: Io, borea: []const u8, file: []const u8, key: []const u8) !?[]const u8 {
    const path = try std.fs.path.join(arena, &.{ borea, file });
    const text = Io.Dir.cwd().readFileAlloc(io, path, arena, .limited(1 << 20)) catch return null;
    var lines = std.mem.splitScalar(u8, text, '\n');
    while (lines.next()) |line| {
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (!std.mem.startsWith(u8, trimmed, key)) continue;
        const rest = std.mem.trim(u8, trimmed[key.len..], " \t");
        if (rest.len == 0 or rest[0] != '=') continue;
        const value = std.mem.trim(u8, rest[1..], " \t");
        if (value.len >= 2 and value[0] == '"' and value[value.len - 1] == '"') return value[1 .. value.len - 1];
    }
    return null;
}
