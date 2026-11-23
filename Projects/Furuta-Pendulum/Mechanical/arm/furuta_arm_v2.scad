// ============================================================
// furuta_arm_v2.scad  —  一体水平臂（薄版）
// 120mm 全长，4mm 薄，主轴端→摆杆端无缝一体
// ============================================================

$fn  = 80;
TOL  = 0.15;

// ---------- 臂 ----------
ARM_LEN = 120;       // 全长 mm
ARM_W   = 14;        // 臂宽
ARM_H   = 4;         // 臂厚

// ---------- 主轴端 ----------
SHAFT_D = 6.0;       // 主轴孔径
BOSS_S  = 13;        // 主轴端方台边长

// ---------- 摆杆端 ----------
BEARING_OD = 10.0;
BEARING_ID = 5.0;
BEARING_H  = 4.0;
BEARING_GAP = 9;     // 双轴承间距
BOSS_P     = 16;     // 摆杆端圆座直径

// ---------- AS5600 ----------
AS_SPAN  = 12.5;
M2_D     = 2.0;
MAG_GAP  = 2.5;

// ---------- 顶丝 ----------
M3_D = 3.0;

// ============================================================

// 主轴端中心 x
SHAFT_X = BOSS_S / 2;
// 摆杆端中心 x
PEND_X  = ARM_LEN - BOSS_P / 2;
// 臂身段 x 范围
BEAM_X1 = SHAFT_X + BOSS_S/2;
BEAM_X2 = PEND_X - BOSS_P/2;
BEAM_MID = (BEAM_X1 + BEAM_X2) / 2;

module furuta_arm_v2() {
    difference() {
        union() {
            // ======== 臂身 (扁梁) ========
            translate([BEAM_MID, 0, 0])
                cube([BEAM_X2 - BEAM_X1, ARM_W, ARM_H], center=true);

            // ======== 主轴端方台 ========
            translate([SHAFT_X, 0, -BOSS_S/2 + ARM_H/2])
                cube([BOSS_S, BOSS_S, BOSS_S], center=true);

            // 方台 → 臂身 渐变
            hull() {
                translate([SHAFT_X + BOSS_S/2 - 0.5, 0, 0])
                    cube([2, BOSS_S, ARM_H], center=true);
                translate([BEAM_X1 + 4, 0, 0])
                    cube([2, ARM_W, ARM_H], center=true);
            }

            // ======== 摆杆端圆柱座 ========
            bh = BEARING_H * 2 + BEARING_GAP;  // 轴承柱总高
            translate([PEND_X, 0, 0])
                cylinder(d=BOSS_P, h=bh, center=true);

            // 圆柱顶底加厚 (防层裂)
            translate([PEND_X, 0,  bh/2]) cylinder(d1=BOSS_P, d2=BOSS_P-4, h=3, center=true);
            translate([PEND_X, 0, -bh/2]) cylinder(d1=BOSS_P, d2=BOSS_P-4, h=3, center=true);

            // 圆柱 → 臂身 渐变
            hull() {
                translate([PEND_X - BOSS_P/2 + 1, 0, 0])
                    cube([2, BOSS_P, ARM_H], center=true);
                translate([BEAM_X2 - 4, 0, 0])
                    cube([2, ARM_W, ARM_H], center=true);
            }

            // ======== AS5600 安装腿 (从圆柱底向下伸) ========
            leg_base = -bh/2 - MAG_GAP - 2;
            for (sx = [-1, 1]) {
                hull() {
                    translate([PEND_X + sx * AS_SPAN/2, 0, leg_base])
                        cylinder(d=5, h=6, center=true);
                    translate([PEND_X + sx * 8, 0, -bh/2])
                        cylinder(d=4, h=2, center=true);
                }
            }
            // 横梁连两腿
            translate([PEND_X, 0, leg_base])
                cube([AS_SPAN + 6, 5, 3], center=true);
        }

        // ======== 主轴孔 ========
        translate([SHAFT_X, 0, 0])
            cylinder(d=SHAFT_D + TOL, h=BOSS_S + 4, center=true);

        // ======== 主轴 M3 顶丝 (从侧面打入) ========
        translate([SHAFT_X, BOSS_S/2, 0])
            rotate([90, 0, 0])
                cylinder(d=M3_D + 0.2, h=10, center=true);

        // ======== 摆杆轴通孔 ========
        translate([PEND_X, 0, 0])
            cylinder(d=BEARING_ID + 0.3, h=bh + 15, center=true);

        // ======== 上轴承腔 ========
        translate([PEND_X, 0, BEARING_GAP/2])
            cylinder(d=BEARING_OD + TOL, h=BEARING_H + 0.4, center=true);

        // ======== 下轴承腔 ========
        translate([PEND_X, 0, -BEARING_GAP/2])
            cylinder(d=BEARING_OD + TOL, h=BEARING_H + 0.4, center=true);

        // ======== 轴承间侧窗 (插摆杆) ========
        translate([PEND_X, 0, 0])
            cube([BOSS_P + 2, BEARING_OD + 4, BEARING_GAP - 1.5], center=true);

        // ======== M2 AS5600 孔 ========
        for (sx = [-1, 1]) {
            translate([PEND_X + sx * AS_SPAN/2, 0, leg_base - 3])
                cylinder(d=M2_D + 0.2, h=10, center=true);
        }
    }
}

rotate([0, 0, 90])
    furuta_arm_v2();
