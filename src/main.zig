const std = @import("std");
const faa = @import("faa_database.zig");
const crc = @import("crc.zig");
const adsb = @import("root.zig");
const Allocator = std.mem.Allocator;

pub fn display_plane(plane: adsb.Plane, timestamp: i64, output: anytype, owner: [64]u8) !void {
    try output.print("{x:06}: {s}: ", .{ plane.icao, plane.callsign });
    const coords: adsb.Coordinates = adsb.get_plane_coordinates(plane);
    // zig fmt: off
    try output.print(
        "Lat: {d:13.8}, Lon: {d:13.8}, Alt: {d:5} | Last heard {d:3}s ago ",
        .{ coords.lat, coords.lon - 0.0, plane.alt, timestamp - plane.ts }
        );
    if (owner[0] == 0) {
        try output.print("| {s} | ", .{ "                                                  " }); 
    } else {
        try output.print("| {s} | ", .{ owner });
    }
    try output.print("{}\n", .{ plane.wvc });
    // zig fmt: on
}

pub fn print_message_details(frame: adsb.Frame, plane: adsb.Plane, output: anytype) !void {
    try output.print("{x:06}: {s}: ", .{ plane.icao, plane.callsign });
    const message: adsb.UnknownMessage = @bitCast(frame.message);
    switch (message.tc) {
        adsb.MessageType.IDENT_FIRST...adsb.MessageType.IDENT_LAST => {
            try output.print("Ident: {s} | ", .{plane.callsign});
            try output.print("{}\n", .{plane.wvc});
        },
        adsb.MessageType.SURFACE_POS_FIRST...adsb.MessageType.SURFACE_POS_LAST => {
            try output.print("Surface Position\n", .{});
        },
        adsb.MessageType.AIR_POS_FIRST...adsb.MessageType.AIR_POS_LAST => {
            // zig fmt: off
            const coords: adsb.Coordinates = adsb.get_plane_coordinates(plane);
            try output.print(
                "AirPos Baro: {d:.8}, {d:.8} | {d:5}\n",
                .{ coords.lat, coords.lon - 0.0, plane.alt }
                );
            // zig fmt: on
        },
        adsb.MessageType.AIRBORNE_VELOCITIES => {
            try output.print("Airborne Velocities\n", .{});
        },
        adsb.MessageType.AIR_POS_GNSS_FIRST...adsb.MessageType.AIR_POS_GNSS_LAST => {
            try output.print("Airborne Position w/ GNSS\n", .{});
        },
        adsb.MessageType.RESERVED_FIRST...adsb.MessageType.RESERVED_LAST => {
            try output.print("RESERVED\n", .{});
        },
        adsb.MessageType.AIRCRAFT_STATUS => {
            try output.print("Aircraft Status\n", .{});
        },
        adsb.MessageType.TARGET_STATE_AND_STATUS_INFO => {
            try output.print("Target state and status info\n", .{});
        },
        adsb.MessageType.AIRCRAFT_OPERATION_STATUS => {
            try output.print("Aircraft operation status\n", .{});
        },
        else => {
            try output.print("Unknown\n", .{});
        },
    }
}

pub fn display_all_planes(planes_table: std.AutoHashMap(u24, adsb.Plane), faa_database_table: std.AutoHashMap(u24, faa.FAAPlane), output: anytype) !void {
    var plane_iter = planes_table.valueIterator();
    try output.print("=============\n", .{});
    while (plane_iter.next()) |p| {
        if (std.time.timestamp() - p.ts >= 60) {
            continue;
        }
        const faa_entry = faa_database_table.get(p.icao);
        var owner: [64]u8 = std.mem.zeroes([64]u8);
        if (faa_entry) |entry| {
            owner = entry.owner;
        }
        try display_plane(p.*, std.time.timestamp(), output, owner);
    }
}

pub fn main() !void {
    const stdin = std.io.getStdIn().reader();
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();

    const verbose: bool = false;

    var general_purpose_allocator = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa = general_purpose_allocator.allocator();

    try stdout.print("Starting adsb.zig...\n", .{});
    try bw.flush();

    try stdout.print("Loading FAA registration database file...", .{});
    try bw.flush();
    var faa_database_table = try faa.load_faa_database("/home/fox/Documents/faa_plane_database/MASTER.txt", gpa);
    defer faa_database_table.deinit();
    try stdout.print(" Done.\n", .{});

    try stdout.print("Generating crc error correction table...", .{});
    try bw.flush();
    var table = try crc.gen_crc24_error_table(gpa);
    defer table.deinit();
    try stdout.print(" Done.\n", .{});

    try stdout.print("Waiting for ADSB frames.\n", .{});
    try bw.flush();

    var planes_table = std.AutoHashMap(u24, adsb.Plane).init(gpa);
    defer planes_table.deinit();

    while (true) {
        var input: [32]u8 = std.mem.zeroes([32]u8);
        _ = stdin.readUntilDelimiter(&input, '\n') catch {
            break;
        };
        const parsed_input: []u8 = input[1..29];

        var frame_raw: u112 = try std.fmt.parseInt(u112, parsed_input, 16);
        const frame_crc: u112 = crc.crc24(frame_raw);
        if (frame_crc != 0) {
            const val = table.get(frame_crc);
            if (val) |v| {
                frame_raw = frame_raw ^ v;
            } else {
                continue;
            }
        }

        const frame: adsb.Frame = @bitCast(frame_raw);
        const plane: adsb.Plane = adsb.convert_frame_to_plane(frame, planes_table);
        if (verbose) {
            try print_message_details(frame, plane, stdout);
        }

        try planes_table.put(frame.icao, plane);
        try display_all_planes(planes_table, faa_database_table, stdout);
        try bw.flush();
    }
    try stdout.print("Goodbye\n", .{});
    try bw.flush();
}
