// Estimated model from product image: "旋转倒立摆弯摆杆"
// Units: mm
// Known from image:
//   overall length ~= 153
//   width ~= 36
//   upright height ~= 41
//   mass ~= 0.14 kg
// Unknown and estimated:
//   plate thickness, hole diameters, hole positions, upright ring hole size

$fn = 64;

length_total = 153;
width = 36;
plate_thickness = 2;
corner_radius = 18;

upright_height = 41;
upright_thickness = 5;
upright_width = 36;
upright_inner_hole_d = 22;
upright_small_hole_d = 3.2;

// Estimated mounting holes on the long arm.
arm_hole_d = 4.2;
large_end_hole_d = 8;

module rounded_rect_2d(l, w, r) {
    hull() {
        translate([r, r]) circle(r = r);
        translate([l - r, r]) circle(r = r);
        translate([r, w - r]) circle(r = r);
        translate([l - r, w - r]) circle(r = r);
    }
}

module rounded_plate(l, w, t, r) {
    linear_extrude(height = t)
        rounded_rect_2d(l, w, r);
}

module arch_tab_2d(h, w) {
    // 2D profile for the upright tab:
    // flat lower block with a large semicircular crown.
    r = w / 2;
    hull() {
        translate([0, r]) circle(r = 0.01);
        translate([h - r, r]) circle(r = r);
        translate([0, w - r]) circle(r = 0.01);
    }
    square([h - r, w]);
}

module base_plate() {
    difference() {
        rounded_plate(length_total, width, plate_thickness, corner_radius);

        // Single hole near upright side, estimated from image.
        translate([42, width / 2, -1])
            cylinder(h = plate_thickness + 2, d = arm_hole_d);

        // Hole cluster on far rounded end, estimated.
        translate([132, width / 2, -1])
            cylinder(h = plate_thickness + 2, d = large_end_hole_d);

        for (p = [[121, 12], [121, 24], [137, 12], [137, 24]]) {
            translate([p[0], p[1], -1])
                cylinder(h = plate_thickness + 2, d = arm_hole_d);
        }
    }
}

module upright_plate() {
    // Upright plate stands near x = 0, extruded along x.
    translate([0, 0, plate_thickness])
    rotate([0, 90, 0])
    linear_extrude(height = upright_thickness)
    difference() {
        arch_tab_2d(upright_height, upright_width);

        // Large central opening.
        translate([upright_height / 2, upright_width / 2])
            circle(d = upright_inner_hole_d);

        // Four small bolt holes, estimated.
        for (p = [[10, 8], [10, 28], [31, 8], [31, 28]]) {
            translate([p[0], p[1]])
                circle(d = upright_small_hole_d);
        }
    }
}

module fillet_block_estimate() {
    // Rounded bend reinforcement estimate.
    translate([0, 0, 0])
        rounded_plate(upright_thickness + 4, width, plate_thickness + 5, 4);
}

union() {
    base_plate();
    upright_plate();
    fillet_block_estimate();
}
