// ============================================================
// furuta_arm.scad  —  一体水平臂
//
// 长 120mm 扁梁，近端固定 6mm 主轴，远端集成：
//   - 双 MR105 轴承座 (5mm ID, 10mm OD)
//   - AS5600 安装位
//   - 5mm 钢轴穿摆杆
// 一次打印，无需铝管
// ============================================================

$fn  = 60;
TOL  = 0.15;

// ---------- 臂尺寸 ----------
ARM_LEN  = 120;        // 臂总长 mm（含轴承端）
ARM_W    = 16;         // 臂宽
ARM_H    = 6;          // 臂厚

// ---------- 主轴端 ----------
SHAFT_D      = 6.0;    // 主轴直径
SHAFT_BOSS_D = 14;     // 主轴处加强环外径
SHAFT_BOSS_H = 10;     // 主轴处加强环高

// ---------- 轴承端 MR105 ----------
BEARING_OD = 10.0;     // 外径
BEARING_ID = 5.0;      // 内径
BEARING_H  = 4.0;      // 单片厚
BEARING_GAP = 9;       // 双轴承间距（摆杆厚度+间隙）
BEARING_BOSS_D = BEARING_OD + 6;  // 轴承座外径

// ---------- 摆杆轴 ----------
PEND_SHAFT_D = 5.0;

// ---------- AS5600 ----------
AS5600_SPAN = 12.5;     // M2 孔距
M2_D = 2.0;
MAG_GAP = 2.5;          // 磁铁到 PCB 间隙

// ---------- 顶丝 ----------
M3_D = 3.0;


// ============================================================

ARM_START = 0;                          // 臂梁起点
ARM_END   = ARM_LEN - BEARING_BOSS_D/2; // 到轴承座中心

module furuta_arm() {
    difference() {
        union() {
            // ---- 臂梁 ----
            translate([ARM_START + SHAFT_BOSS_D/2, 0, 0])
                cube([ARM_LEN - SHAFT_BOSS_D/2 - BEARING_BOSS_D/2, ARM_W, ARM_H], center=true);

            // ---- 主轴加强环 ----
            translate([SHAFT_BOSS_D/2, 0, -SHAFT_BOSS_H/2 + ARM_H/2])
                cylinder(d=SHAFT_BOSS_D, h=SHAFT_BOSS_H, center=true);

            // ---- 臂 → 主轴环 过渡三角 ----
            hull() {
                translate([SHAFT_BOSS_D/2 + 2, 0, 0])
                    cube([ARM_W + 4, ARM_W, ARM_H], center=true);
                translate([SHAFT_BOSS_D - 2, 0, 0])
                    cylinder(d=SHAFT_BOSS_D, h=ARM_H, center=true);
            }

            // ---- 轴承座圆柱 ----
            translate([ARM_END, 0, 0])
                cylinder(d=BEARING_BOSS_D, h=BEARING_H*2 + BEARING_GAP, center=true);

            // ---- 臂 → 轴承座 过渡三角 ----
            hull() {
                translate([ARM_END - BEARING_BOSS_D/2 - 2, 0, 0])
                    cube([ARM_W + 4, ARM_W, ARM_H], center=true);
                translate([ARM_END - BEARING_BOSS_D + 2, 0, 0])
                    cylinder(d=BEARING_BOSS_D, h=ARM_H, center=true);
            }

            // ---- 轴承座顶部加强 (防裂) ----
            translate([ARM_END, 0, BEARING_H + BEARING_GAP/2])
                cylinder(d=BEARING_BOSS_D, h=3, center=true);
            translate([ARM_END, 0, -BEARING_H - BEARING_GAP/2])
                cylinder(d=BEARING_BOSS_D, h=3, center=true);

            // ---- AS5600 安装耳 ----
            translate([ARM_END, 0, -BEARING_H*2 - BEARING_GAP/2 - MAG_GAP - 3])
                for (x = [-AS5600_SPAN/2, AS5600_SPAN/2])
                    translate([x, 0, 0])
                        cylinder(d=6, h=8, center=true);

            // ---- AS5600 耳横梁 ----
            translate([ARM_END, 0, -BEARING_H*2 - BEARING_GAP/2 - MAG_GAP - 3])
                cube([AS5600_SPAN + 6, BEARING_OD + 2, 3], center=true);
        }

        // ---- 主轴孔 ----
        translate([SHAFT_BOSS_D/2, 0, 0])
            cylinder(d=SHAFT_D + TOL, h=SHAFT_BOSS_H + 10, center=true);

        // ---- 主轴 M3 顶丝 (侧面) ----
        translate([SHAFT_BOSS_D/2, ARM_W/2 + 3, 0])
            rotate([90, 0, 0])
                cylinder(d=M3_D + 0.2, h=10, center=true);

        // ---- 摆杆轴通孔 ----
        translate([ARM_END, 0, 0])
            cylinder(d=PEND_SHAFT_D + 0.3, h=BEARING_H*2 + BEARING_GAP + 12, center=true);

        // ---- 上轴承腔 ----
        translate([ARM_END, 0, BEARING_GAP/2])
            cylinder(d=BEARING_OD + TOL, h=BEARING_H + 0.4, center=true);

        // ---- 下轴承腔 ----
        translate([ARM_END, 0, -BEARING_GAP/2])
            cylinder(d=BEARING_OD + TOL, h=BEARING_H + 0.4, center=true);

        // ---- 两轴承间侧向开口 (摆杆安装窗) ----
        translate([ARM_END, 0, 0])
            cube([ARM_W + 4, BEARING_OD + 4, BEARING_GAP - 1.5], center=true);

        // ---- M2 AS5600 孔 ----
        for (x = [-AS5600_SPAN/2, AS5600_SPAN/2]) {
            translate([ARM_END + x, 0, -BEARING_H*2 - BEARING_GAP/2 - MAG_GAP - 8])
                cylinder(d=M2_D + 0.2, h=12, center=true);
        }
    }
}

rotate([0, 0, 90])
    furuta_arm();


// ============================================================
// 安装:
//   1. 近端大孔套在 6mm 主轴上，侧面 M3 顶丝锁死
//   2. 远端上/下各压入 MR105 轴承
//   3. 5mm 钢轴从上穿过上轴承 → 摆杆孔 → 下轴承
//   4. 摆杆 (铝条打 5mm 孔) 从侧面插入两轴承之间
//   5. 磁铁粘轴底，AS5600 用 M2 螺丝固定
//
// 清单替换:
//   #5 铝管+#6 管座+#7 端座 → 这一个打印件 (¥8-12)
// ============================================================
