const std = @import("std");
const Allocator = std.mem.Allocator;

pub const FAAPlane = struct {
    icao: u24,
    owner: [64]u8,
};

pub fn load_faa_database(absolute_path: []const u8, allocator: Allocator) !std.AutoHashMap(u24, FAAPlane) {
    var faa_database_table = std.AutoHashMap(u24, FAAPlane).init(allocator);

    var file = try std.fs.openFileAbsolute(absolute_path, .{});
    defer file.close();

    var buf_reader = std.io.bufferedReader(file.reader());
    var in_stream = buf_reader.reader();

    var buf: [1024]u8 = undefined;
    var first_line: bool = true;
    while (try in_stream.readUntilDelimiterOrEof(&buf, '\n')) |line| {
        if (first_line) {
            first_line = false;
            continue;
        }
        var it = std.mem.splitSequence(u8, line, ",");
        _ = it.first();

        // Skip the next 5
        var j: u8 = 0;
        while (j < 5) : (j += 1) {
            _ = it.next().?;
        }

        const next = it.next().?;
        var owner: [64]u8 = std.mem.zeroes([64]u8);
        for (0.., next[0..@min(64, next.len)]) |i, elem| {
            owner[i] = elem;
        }

        // Skip the next 26
        j = 0;
        while (j < 26) : (j += 1) {
            _ = it.next().?;
        }

        const icao: u24 = try std.fmt.parseInt(u24, it.next().?[0..6], 16);
        const faaPlane: FAAPlane = FAAPlane{
            .icao = icao,
            .owner = owner,
        };
        try faa_database_table.put(icao, faaPlane);
    }

    return faa_database_table;
}
