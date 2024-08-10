const std = @import("std");
const Allocator = std.mem.Allocator;

const WakeVortexCategory = enum {
    RESERVED, NO_CATEGORY_INFO,
    GROUND_OBSTRUCTION, SURFACE_EMERGENCY_VEHICLE, SURFACE_SERVICE_VEHICLE,
    GLIDER, LIGHTER_THAN_AIR, SKYDIVER, ULTRALIGHT, UAV, TRANSATMOSPHERIC_VEHICLE,
    LIGHT, MEDIUM1, MEDIUM2, HIGH_VORTEX_AIRCRAFT, HEAVY, HIGH_PERFORMANCE, ROTORCRAFT
};

const IdentMessage = packed struct{
    c8: u6, c7: u6, c6: u6, c5: u6, c4: u6,
    c3: u6, c2: u6, c1: u6, ca: u3, tc: u5,
};
const SurPosMessage = packed struct {
    lon_cpr: u17, lat_cpr: u17,
    f: u1, t: u1, trk: u7, s: u1, mov: u7, tc: u5,
};
const AirPosMessage = packed struct {
    lon_cpr: u17, lat_cpr: u17,
    f: u1, t: u1, alt: u12, saf: u1, ss: u2, tc: u5,
};
const AirVelMessage = packed struct {
    junk: u51, tc: u5, // TODO: fill out
};
const OpStatusMessage = packed struct {
    junk: u51, tc: u5, // TODO: fill out
};
const UnknownMessage = packed struct {
    junk: u51, tc: u5,
};

const Frame = packed struct{
    pi: u24,
    message: u56,
    icao: u24,
    ca: u3,
    df: u5,
};

const Plane = struct{
    icao: u24,
    callsign: [8]u8,
    lat_even: u17,
    lon_even: u17,
    lat_odd: u17,
    lon_odd: u17,
    alt: u32,
    has_frame_0: bool,
    has_frame_1: bool,
    ts: i64,
};

fn crc24(data: u112) u112 {
    var result: u112 = data;
    const generator: u112 = 0x1fff409;
    for (24..112) |i| {
        const index: u7 = 111-(@as(u7, @intCast(i))-24);
        if (result >> index & 1 != 0){
            result = result ^ (generator << index - 24);
        }
    }
    return result;
}

// TODO: Make this be able to check for more than one error bit.
fn gen_crc24_error_table(allocator: Allocator) !std.AutoHashMap(u112, u112) {
    var table = std.AutoHashMap(u112, u112).init(allocator);
    var check_error: u112 = 0;

    for (0..112) |i|{
        const index: u7 = 111-@as(u7, @intCast(i));
        const one: u112 = 1;
        check_error = one << index;
        const syndrome: u112 = crc24(check_error);

        try table.putNoClobber(syndrome, one << index);
    }

    return table;
}

fn calc_lat(lat_cpr_even_raw: u17, lat_cpr_odd_raw: u17, is_odd: bool) f64 {
    const lat_cpr_even: f64 = @as(f64, @floatFromInt(lat_cpr_even_raw))/131072.0;
    const lat_cpr_odd: f64 = @as(f64, @floatFromInt(lat_cpr_odd_raw))/131072.0;

    const Nz: f64 = 15.0;
    const d_lat_even: f64 = 360 / (4*Nz);
    const d_lat_odd: f64 = 360 / (4*Nz - 1.0);

    const lat_zone_index = @floor(59.0 * lat_cpr_even - 60.0 * lat_cpr_odd + 0.5);

    var result: f64 = 0.0;
    if (is_odd){
        result = d_lat_odd * (@mod(lat_zone_index, 59) + lat_cpr_odd);
    }else{
        result = d_lat_even * (@mod(lat_zone_index, 60) + lat_cpr_even);
    }
    if (result >= 270.0){
        result = result - 360.0;
    }
    return result;
}

fn calc_nl(lat: f64) f64 {
    if (lat > 87.0){ return 1.0; }
    if (lat < -87.0){ return 1.0; }
    const PI: f64 = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;
    const Nz: f64 = 15.0;
    var result: f64 = (PI / 180) * lat;
    result = 0.5 * (1.0 + @cos(2.0 * result));
    result = 1.0 - ((1.0 - @cos(PI / (2 * Nz))) / result);
    result = @floor((2 * PI) / std.math.acos(result));
    return result;
}

fn calc_lon(lat: f64, lon_cpr_even: u17, lon_cpr_odd: u17, is_odd: bool) f64 {
    const m: f64 = @floor((@as(f64, @floatFromInt(lon_cpr_even))/131072.0 * (calc_nl(lat) - 1.0)) - (@as(f64, @floatFromInt(lon_cpr_odd))/131072.0 * calc_nl(lat)) + 0.5);
    var offset: f64 = 0.0;
    if (is_odd){ offset = 1.0; }
    const n: f64 = @max(calc_nl(lat - offset), 1.0);
    if (is_odd){
        return 360.0/n * ((@mod(m, n) + @as(f64, @floatFromInt(lon_cpr_odd))/131072.0));
    }else{
        return 360.0/n * ((@mod(m, n) + @as(f64, @floatFromInt(lon_cpr_even))/131072.0));
    }
}

pub fn main() !void {
    const verbose: bool = false;
    const stdin = std.io.getStdIn().reader();
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();
    var bad_msgs: usize = 0;

    var general_purpose_allocator = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa = general_purpose_allocator.allocator();
    var table = try gen_crc24_error_table(gpa);
    defer table.deinit();

    var planes_table = std.AutoHashMap(u24, Plane).init(gpa);
    defer planes_table.deinit();
    try stdout.print("Starting adsb.zig\n", .{});

    while (true){
        var input: [32]u8 = std.mem.zeroes([32]u8);
        _ = stdin.readUntilDelimiter(&input, '\n') catch {
            break;
        };
        const parsed_input: []u8 = input[1..29];
        //try stdout.print("{s}\n", .{parsed_input});

        var frame_raw: u112 = try std.fmt.parseInt(u112, parsed_input, 16);
        const frame_crc: u112 = crc24(frame_raw);
        if (frame_crc != 0){
            const val = table.get(frame_crc);
            if (val) |v| {
                frame_raw = frame_raw ^ v;
                if (verbose){
                    if (bad_msgs > 0){
                        try stdout.print("\n", .{});
                    }
                    try stdout.print("-: ", .{});
                }
            }else{
                if (verbose){
                    try stdout.print("x", .{});
                    try bw.flush();
                }
                bad_msgs += 1;
                continue;
            }
        }else{
            if (verbose){
                if (bad_msgs > 0){
                    try stdout.print("\n", .{});
                }
                try stdout.print("✓: ", .{});
            }
        }
        bad_msgs = 0;
        const frame: Frame = @bitCast(frame_raw);

        var plane: Plane = Plane{
            .icao = frame.icao,
            .callsign = [8]u8{'#', '#', '#', '#', '#', '#', '#', '#',},
            .lat_even = 0, .lon_even = 0,
            .lat_odd = 0, .lon_odd = 0,
            .alt = 0,
            .has_frame_0 = false, .has_frame_1 = false,
            .ts = 0,
        };
        const plane_optional = planes_table.get(frame.icao);
        if (plane_optional)|p|{
            plane = p;
        }

        const msg: UnknownMessage = @bitCast(frame.message);
        if (verbose){
            try stdout.print("{x:06}: {s}: ", .{plane.icao, plane.callsign});
        }
        plane.ts = std.time.timestamp();
        switch (msg.tc) {
            1...4 => {
                const msg2: IdentMessage = @bitCast(frame.message);
                const mapping: *const [64:0]u8 = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
                plane.callsign = [8]u8{mapping[msg2.c1], mapping[msg2.c2], mapping[msg2.c3], mapping[msg2.c4], mapping[msg2.c5], mapping[msg2.c6], mapping[msg2.c7], mapping[msg2.c8]};
                const wvc_code: u8 = @as(u8, @intCast(msg2.tc - 1)) << 3 | @as(u8, @intCast(msg2.ca));
                const wake_vortex_category: WakeVortexCategory = switch(wvc_code){
                    0b00000...0b00111 => .RESERVED,
                    0b10101 => .RESERVED,
                    0b01000 => .NO_CATEGORY_INFO,
                    0b10000 => .NO_CATEGORY_INFO,
                    0b11000 => .NO_CATEGORY_INFO,
                    0b01010 => .NO_CATEGORY_INFO,

                    0b01100...0b01111 => .GROUND_OBSTRUCTION,
                    0b01001 => .SURFACE_EMERGENCY_VEHICLE,
                    0b01011 => .SURFACE_SERVICE_VEHICLE,

                    0b10001 => .GLIDER,
                    0b10010 => .LIGHTER_THAN_AIR,
                    0b10011 => .SKYDIVER,
                    0b10100 => .ULTRALIGHT,
                    0b10110 => .UAV,
                    0b10111 => .TRANSATMOSPHERIC_VEHICLE,

                    0b11001 => .LIGHT,
                    0b11010 => .MEDIUM1,
                    0b11011 => .MEDIUM2,
                    0b11100 => .HIGH_VORTEX_AIRCRAFT,
                    0b11101 => .HEAVY,
                    0b11110 => .HIGH_PERFORMANCE,
                    0b11111 => .ROTORCRAFT,
                    else => .NO_CATEGORY_INFO,
                };
                if (verbose){
                    try stdout.print("Ident: {s} | ", .{plane.callsign});
                    try stdout.print("{}\n", .{wake_vortex_category});
                }
            },
            5...8 => {if(verbose){ try stdout.print("Surface Position\n", .{}); }},
            9...18 => {
                const msg2: AirPosMessage = @bitCast(frame.message);
                const q_bit: bool = (msg2.alt >> 4 & 1) == 1;
                _ = q_bit; // TODO: Handle q_bit
                // TODO: If q_bit is false, handle gray code.
                var actual_alt: u32 = @intCast(((msg2.alt >> 1) & 0b11111110000) | (msg2.alt & 0b1111));
                actual_alt = (actual_alt * 25) - 1000;
                plane.alt = actual_alt;

                if (msg2.f == 0){
                    plane.lat_even = msg2.lat_cpr;
                    plane.lon_even = msg2.lon_cpr;
                    plane.has_frame_0 = true;
                }else{
                    plane.lat_odd = msg2.lat_cpr;
                    plane.lon_odd = msg2.lon_cpr;
                    plane.has_frame_1 = true;
                }
                if (verbose){
                    if (plane.has_frame_0 and plane.has_frame_1){
                        const lat: f64 = calc_lat(plane.lat_even, plane.lat_odd, false);
                        const lon: f64 = calc_lon(lat, plane.lon_even, plane.lon_odd, false);
                        try stdout.print("AirPos Baro: {d:.8}, {d:.8} | {d:5}\n", .{lat, lon - 360.0, plane.alt});
                    }else{
                        try stdout.print("AirPos Baro: Not yet valid lon/lat!    | {d:5}\n", .{plane.alt});
                    }
                }
            },
            19 => {if (verbose){try stdout.print("Airborne Velocities\n", .{});}},
            20...22 => {if (verbose){try stdout.print("Airborne Position w/ GNSS\n", .{});}},
            23...27 => {if (verbose){try stdout.print("RESERVED\n", .{});}},
            28 => {if (verbose){try stdout.print("Aircraft Status\n", .{});}},
            29 => {if (verbose){try stdout.print("Target state and status info\n", .{});}},
            31 => {if (verbose){try stdout.print("Aircraft operation status\n", .{});}},
            else => {if (verbose){try stdout.print("Unknown\n", .{});}},
        }
        //try stdout.print("{}\n", .{msg});
        try planes_table.put(frame.icao, plane);
        var plane_iter = planes_table.valueIterator();
        try stdout.print("=============\n", .{});
        while (plane_iter.next()) |p| {
            if (std.time.timestamp() - p.ts >= 60){
                continue;
            }
            try stdout.print("{x:06}: {s}: ", .{p.icao, p.callsign});
            if (p.has_frame_0 and p.has_frame_1){
                //const my_lat: f64 = 0.0;
                //const my_lon: f64 = 0.0 + 360.0;
                const lat: f64 = calc_lat(p.lat_even, p.lat_odd, false);
                const lon: f64 = calc_lon(lat, p.lon_even, p.lon_odd, false);
                //const dist: f64 = std.math.sqrt(std.math.pow(f64, my_lat - lat, 2.0) + std.math.pow(f64, my_lon - lon, 2.0));
                try stdout.print("Lat: {d:.8}, Lon: {d:.8}, Alt: {d:5} | Last heard {d}s ago\n", .{lat, lon - 360.0, p.alt, std.time.timestamp() - p.ts});
            }else{
                try stdout.print("Not yet valid lon/lat!    | Alt: {d:5} | Last heard {d}s ago\n", .{p.alt, std.time.timestamp() - p.ts});
            }
        }
        try bw.flush();
    }
    if (bad_msgs > 0){
        try stdout.print("\n", .{});
    }
    try stdout.print("Goodbye\n", .{});
    try bw.flush();
}
