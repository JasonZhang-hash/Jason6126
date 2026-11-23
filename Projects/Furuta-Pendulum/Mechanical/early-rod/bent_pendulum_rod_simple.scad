// Simple clean model requested by Jason.
// Units: mm
// Shape:
//   1. One long thin rectangular plate.
//   2. At the tail, one upright tab: small rectangle + semicircle top.
//   3. No rounded corners, no holes, no fillets.

$fn = 72;

plate_length = 153;
plate_width = 36;
plate_thickness = 2;

tab_thickness = 5;       // extrusion along plate length
tab_width = 36;          // same as plate width
tab_rect_height = 23;    // rectangular lower part
tab_radius = tab_width / 2;
tab_total_height = tab_rect_height + tab_radius;

module base_plate() {
    cube([plate_length, plate_width, plate_thickness]);
}

module tab_profile_2d() {
    // Coordinates are [y, z].
    // Lower rectangle from z=0 to tab_rect_height.
    square([tab_width, tab_rect_height]);

    // Semicircle sitting on top of rectangle.
    translate([tab_width / 2, tab_rect_height])
        difference() {
            circle(r = tab_radius);
            translate([-tab_width, -tab_radius * 2])
                square([tab_width * 2, tab_radius * 2]);
        }
}

module upright_tab() {
    // Place tab at tail of plate, standing on the plate top.
    translate([0, 0, plate_thickness])
        rotate([90, 0, 90])
            linear_extrude(height = tab_thickness)
                tab_profile_2d();
}

union() {
    base_plate();
    upright_tab();
}
