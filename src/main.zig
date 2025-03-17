const std = @import("std");
const faa = @import("faa_database.zig");
const crc = @import("crc.zig");
const adsb = @import("root.zig");
const adsb_debug = @import("debug_display.zig");
const Allocator = std.mem.Allocator;

pub fn main() !void {
    const stdin = std.io.getStdIn().reader();
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();

    const verbose: bool = false;

    var general_purpose_allocator = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa = general_purpose_allocator.allocator();

    try stdout.print("{s}\n", .{std.os.argv});

    // Do all the loading bits
    try stdout.print("Starting adsb.zig...\n", .{});
    try bw.flush();

    // The first parameter the user passes into adsb_decode.zig will be read
    // as the FAA MASTER.txt plane database.

    // TODO: Use command line flags instead
    // TODO: Sanitize command line arguments
    var faa_database_table: std.AutoHashMap(u24, faa.FAAPlane) = undefined;
    if (std.os.argv.len > 1) {
        try stdout.print("Loading FAA registration database file...", .{});
        try bw.flush();
        faa_database_table = try faa.load_faa_database(std.os.argv[1][0..std.mem.len(std.os.argv[1])], gpa);
        try stdout.print(" Done.\n", .{});
    } else {
        faa_database_table = std.AutoHashMap(u24, faa.FAAPlane).init(gpa);
    }
    defer faa_database_table.deinit();

    try stdout.print("Generating crc error correction table...", .{});
    try bw.flush();
    var table = try crc.gen_crc24_error_table(gpa);
    defer table.deinit();
    try stdout.print(" Done.\n", .{});

    try stdout.print("Waiting for ADSB frames.\n", .{});
    try bw.flush();

    var planes_table = std.AutoHashMap(u24, adsb.Plane).init(gpa);
    defer planes_table.deinit();

    // Main loop
    while (true) {
        // Load hexadecimal input from stdin
        var input: [32]u8 = std.mem.zeroes([32]u8);
        _ = stdin.readUntilDelimiter(&input, '\n') catch {
            break;
        };
        const parsed_input: []u8 = input[1..29];

        // Error check and correct
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

        // Handle frame
        const frame: adsb.Frame = @bitCast(frame_raw);
        const plane: adsb.Plane = adsb.convert_frame_to_plane(frame, planes_table);
        if (verbose) {
            try adsb_debug.print_message_details(frame, plane, stdout);
        }

        // Update table and display
        try planes_table.put(frame.icao, plane);
        try adsb_debug.display_all_planes(planes_table, faa_database_table, stdout);
        try bw.flush();
    }
    try stdout.print("Goodbye\n", .{});
    try bw.flush();
}
