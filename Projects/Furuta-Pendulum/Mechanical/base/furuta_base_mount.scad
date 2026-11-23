// Furuta pendulum base mount prototype for Jason Zhang
// Units: mm

$fn = 72;

plate_size = 70;
plate_thickness = 6;
deck_gap = 70;
corner_radius = 5;

standoff_d = 6;
standoff_inset = 8;
standoff_screw_d = 3.2;

// Top bearing block: GFL06 / 6 mm shaft flange estimate
top_center_hole_d = 12;
top_bearing_hole_d = 4.8;
top_bearing_pitch = 35;

// Bottom 25 mm motor bracket hole pattern from supplied drawing
motor_center_hole_d = 9.8;
motor_mount_hole_d = 3.3;
motor_top_pitch_x = 16;
motor_top_y = 4.0;
motor_bottom_pitch_x = 14;
motor_bottom_y = -9.5;

module rounded_square_2d(size, r) {
    hull() {
        translate([r, r]) circle(r = r);
        translate([size - r, r]) circle(r = r);
        translate([r, size - r]) circle(r = r);
        translate([size - r, size - r]) circle(r = r);
    }
}

module plate_with_holes(z, top_plate=false, bottom_plate=false) {
    translate([-plate_size / 2, -plate_size / 2, z])
    difference() {
        linear_extrude(height = plate_thickness)
            rounded_square_2d(plate_size, corner_radius);

        // Four through holes for the standoffs.
        for (x = [standoff_inset, plate_size - standoff_inset])
            for (y = [standoff_inset, plate_size - standoff_inset])
                translate([x, y, -1])
                    cylinder(h = plate_thickness + 2, d = standoff_screw_d);

        if (top_plate) {
            // Bearing center and flange holes, centered on top deck.
            translate([plate_size / 2, plate_size / 2, -1])
                cylinder(h = plate_thickness + 2, d = top_center_hole_d);
            translate([plate_size / 2, plate_size / 2 - top_bearing_pitch / 2, -1])
                cylinder(h = plate_thickness + 2, d = top_bearing_hole_d);
            translate([plate_size / 2, plate_size / 2 + top_bearing_pitch / 2, -1])
                cylinder(h = plate_thickness + 2, d = top_bearing_hole_d);
        }

        if (bottom_plate) {
            // 25 mm motor bracket: center hole plus four M3-ish mounting holes.
            translate([plate_size / 2, plate_size / 2, -1])
                cylinder(h = plate_thickness + 2, d = motor_center_hole_d);
            for (x = [-motor_top_pitch_x / 2, motor_top_pitch_x / 2])
                translate([plate_size / 2 + x, plate_size / 2 + motor_top_y, -1])
                    cylinder(h = plate_thickness + 2, d = motor_mount_hole_d);
            for (x = [-motor_bottom_pitch_x / 2, motor_bottom_pitch_x / 2])
                translate([plate_size / 2 + x, plate_size / 2 + motor_bottom_y, -1])
                    cylinder(h = plate_thickness + 2, d = motor_mount_hole_d);
        }
    }
}

module standoff(x, y) {
    translate([x, y, plate_thickness - 0.1])
    difference() {
        cylinder(h = deck_gap + 0.2, d = standoff_d);
        translate([0, 0, -1])
            cylinder(h = deck_gap + 2.2, d = standoff_screw_d);
    }
}

union() {
    plate_with_holes(0, bottom_plate=true);
    plate_with_holes(plate_thickness + deck_gap, top_plate=true);

    for (x = [-plate_size / 2 + standoff_inset, plate_size / 2 - standoff_inset])
        for (y = [-plate_size / 2 + standoff_inset, plate_size / 2 - standoff_inset])
            standoff(x, y);
}
