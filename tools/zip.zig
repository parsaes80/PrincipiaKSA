//! zip <output.zip> (<name> <file>)...
//!
//! A zip of the files under the given names ('/' separated), stored
//! uncompressed with a fixed date, so the same files give the same zip.

const std = @import("std");
const Io = std.Io;

const Entry = struct { name: []const u8, crc: u32, size: u32, offset: u32, executable: bool };

pub fn main(init: std.process.Init) !void {
    const arena = init.arena.allocator();
    const io = init.io;
    const args = try init.minimal.args.toSlice(arena);
    if (args.len < 2 or (args.len - 2) % 2 != 0) {
        std.debug.print("usage: zip <output.zip> (<name> <file>)...\n", .{});
        return error.Usage;
    }
    const cwd = Io.Dir.cwd();
    var output = try cwd.createFile(io, args[1], .{});
    defer output.close(io);
    var buffer: [1 << 16]u8 = undefined;
    var file_writer = output.writer(io, &buffer);
    const w = &file_writer.interface;

    var entries: std.ArrayList(Entry) = .empty;
    var offset: u32 = 0;
    var i: usize = 2;
    while (i < args.len) : (i += 2) {
        const name = args[i];
        const data = try cwd.readFileAlloc(io, args[i + 1], arena, .unlimited);
        const entry: Entry = .{
            .name = name,
            .crc = std.hash.Crc32.hash(data),
            .size = @intCast(data.len),
            .offset = offset,
            .executable = std.mem.endsWith(u8, name, ".so") or std.mem.endsWith(u8, name, ".dll") or
                std.mem.endsWith(u8, name, ".exe"),
        };
        try entries.append(arena, entry);
        try localHeader(w, entry);
        try w.writeAll(data);
        offset += 30 + @as(u32, @intCast(name.len)) + entry.size;
    }

    const directory_offset = offset;
    var directory_size: u32 = 0;
    for (entries.items) |entry| {
        try centralHeader(w, entry);
        directory_size += 46 + @as(u32, @intCast(entry.name.len));
    }
    try w.writeInt(u32, 0x06054b50, .little); // End of central directory.
    try w.writeInt(u16, 0, .little);
    try w.writeInt(u16, 0, .little);
    try w.writeInt(u16, @intCast(entries.items.len), .little);
    try w.writeInt(u16, @intCast(entries.items.len), .little);
    try w.writeInt(u32, directory_size, .little);
    try w.writeInt(u32, directory_offset, .little);
    try w.writeInt(u16, 0, .little);
    try w.flush();
}

const utf8_names = 0x0800;
const date_1980_01_01 = (1 << 5) | 1;

fn localHeader(w: *Io.Writer, entry: Entry) !void {
    try w.writeInt(u32, 0x04034b50, .little);
    try w.writeInt(u16, 20, .little); // Version needed: 2.0.
    try w.writeInt(u16, utf8_names, .little);
    try w.writeInt(u16, 0, .little); // Stored.
    try w.writeInt(u16, 0, .little); // Time.
    try w.writeInt(u16, date_1980_01_01, .little);
    try w.writeInt(u32, entry.crc, .little);
    try w.writeInt(u32, entry.size, .little);
    try w.writeInt(u32, entry.size, .little);
    try w.writeInt(u16, @intCast(entry.name.len), .little);
    try w.writeInt(u16, 0, .little);
    try w.writeAll(entry.name);
}

fn centralHeader(w: *Io.Writer, entry: Entry) !void {
    try w.writeInt(u32, 0x02014b50, .little);
    try w.writeInt(u16, (3 << 8) | 20, .little); // Made by Unix (permissions), 2.0.
    try w.writeInt(u16, 20, .little);
    try w.writeInt(u16, utf8_names, .little);
    try w.writeInt(u16, 0, .little);
    try w.writeInt(u16, 0, .little);
    try w.writeInt(u16, date_1980_01_01, .little);
    try w.writeInt(u32, entry.crc, .little);
    try w.writeInt(u32, entry.size, .little);
    try w.writeInt(u32, entry.size, .little);
    try w.writeInt(u16, @intCast(entry.name.len), .little);
    try w.writeInt(u16, 0, .little); // Extra.
    try w.writeInt(u16, 0, .little); // Comment.
    try w.writeInt(u16, 0, .little); // Disk.
    try w.writeInt(u16, 0, .little); // Internal attributes.
    const mode: u32 = if (entry.executable) 0o100755 else 0o100644;
    try w.writeInt(u32, mode << 16, .little);
    try w.writeInt(u32, entry.offset, .little);
    try w.writeAll(entry.name);
}
