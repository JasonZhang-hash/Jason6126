// ============================================================
// furuta_arm_v3.scad  —  极简一体臂
// 120mm × 16mm × 5mm 平板
// 近端: 6mm中心孔 + 4×M3法兰孔 (螺栓锁住主轴法兰)
// 远端: 竖直支架, 轴承孔 + AS5600 安装孔
// ============================================================

$fn  = 80;
TOL  = 0.15;

// ---------- 臂 ----------
ARM_LEN = 120;
ARM_W   = 16;
ARM_H   = 5;

// ---------- 近端 ----------
SHAFT_D     = 6.0;      // 主轴孔 (6mm光轴, 经4→6联轴器接电机)
FLANGE_R    = 10;       // 法兰螺栓分布半径
FLANGE_M3_D = 3.0;

// ---------- 远端 ----------
BEARING_D   = 10.0;     // 轴承外径 (MR105)
SHAFT_P_D   = 5.0;      // 摆杆轴径 (轴承内径)
BEARING_H   = 4.0;      // 轴承厚
GAP         = 9;        // 双轴承间距

BRACKET_W   = 20;       // 支架宽
BRACKET_H   = 4;        // 支架壁厚
POST_H      = BEARING_H*2 + GAP + 4;  // 支架总高

// ---------- AS5600 ----------
AS_SPAN  = 12.5;
M2_D     = 2.0;
MAG_GAP  = 2.5;

// ---------- 坐标 ----------
PROX_X = FLANGE_R + 4;              // 近端中心 x
DIST_X = ARM_LEN - BRACKET_W/2;     // 远端中心 x

module furuta_arm_v3() {

    difference() {
        union() {
            // ==== 臂板 ====
            translate([ARM_LEN/2, 0, 0])
                cube([ARM_LEN, ARM_W, ARM_H], center=true);

            // ==== 近端圆盘 (法兰区略微加厚) ====
            translate([PROX_X, 0, ARM_H/2])
                cylinder(d=FLANGE_R*2 + 6, h=1, center=true);

            // ==== 远端竖直支架 ====
            // 底部基座(加宽)
            translate([DIST_X, 0, ARM_H/2])
                cube([BRACKET_W, BRACKET_W, 2], center=true);

            // 竖板 (前后两片, 中间留给摆杆)
            for (sy = [-1, 1]) {
                translate([DIST_X, sy * (BEARING_D/2 + BRACKET_H/2), (POST_H + ARM_H)/2])
                    cube([BRACKET_W, BRACKET_H, POST_H], center=true);
            }

            // 顶部横梁
            translate([DIST_X, 0, POST_H + ARM_H/2])
                cube([BRACKET_W, BRACKET_W, 3], center=true);

            // ==== AS5600 底座 (底部横梁+两耳) ====
            as_z = -MAG_GAP - 2;
            translate([DIST_X, 0, as_z])
                cube([AS_SPAN + 6, BRACKET_H, 3], center=true);
            for (sx = [-1, 1]) {
                translate([DIST_X + sx * AS_SPAN/2, 0, as_z - 2])
                    cylinder(d=6, h=4, center=true);
            }
        }

        // ==== 近端中心孔 ====
        translate([PROX_X, 0, -1])
            cylinder(d=SHAFT_D + TOL, h=ARM_H + 6);

        // ==== M3 顶丝孔 (从臂侧面自攻，顶住6mm光轴) ====
        translate([PROX_X, ARM_W/2, ARM_H/2])
            rotate([0, -90, 0])
                cylinder(d=2.7, h=ARM_W, center=true);

        // ==== 4×M3 法兰螺栓孔 ====
        for (i = [0:3]) {
            ang = i * 90;
            translate([PROX_X + FLANGE_R * cos(ang), FLANGE_R * sin(ang), -1])
                cylinder(d=FLANGE_M3_D + 0.2, h=ARM_H + 6);
        }

        // ==== 轴承孔 ====
        for (z = [-1, 1]) {
            translate([DIST_X, 0, (POST_H + ARM_H)/2 + z * GAP/2])
                rotate([0, 90, 0])
                    cylinder(d=BEARING_D + TOL, h=BRACKET_W + 2, center=true);
        }

        // ==== 摆杆轴通孔 (坚直贯通) ====
        translate([DIST_X, 0, -1])
            cylinder(d=SHAFT_P_D + 0.3, h=POST_H + ARM_H + as_z + 10);

        // ==== 轴承间侧窗 ====
        translate([DIST_X, 0, (POST_H + ARM_H)/2])
            cube([BRACKET_W + 2, BEARING_D + 4, GAP - 1], center=true);

        // ==== M2 AS5600 ====
        for (sx = [-1, 1]) {
            translate([DIST_X + sx * AS_SPAN/2, 0, as_z - 4])
                cylinder(d=M2_D + 0.2, h=8);
        }
    }
}

rotate([0, 0, 90])
    furuta_arm_v3();
