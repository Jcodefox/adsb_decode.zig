const std = @import("std");
const Allocator = std.mem.Allocator;

pub fn crc24(data: u112) u112 {
    var result: u112 = data;
    const generator: u112 = 0x1fff409;
    for (24..112) |i| {
        const index: u7 = 111 - (@as(u7, @intCast(i)) - 24);
        if (result >> index & 1 != 0) {
            result = result ^ (generator << index - 24);
        }
    }
    return result;
}

// TODO: Make this be able to check for more than one error bit.
pub fn gen_crc24_error_table(allocator: Allocator) !std.AutoHashMap(u112, u112) {
    var table = std.AutoHashMap(u112, u112).init(allocator);
    var check_error: u112 = 0;

    for (0..112) |i| {
        const index: u7 = 111 - @as(u7, @intCast(i));
        const one: u112 = 1;
        check_error = one << index;
        const syndrome: u112 = crc24(check_error);

        try table.putNoClobber(syndrome, one << index);
    }

    return table;
}
