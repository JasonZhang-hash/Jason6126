// ============================================================
// arm_end_mount.scad  —  Furuta 臂端座
// 功能：8mm铝管插座 + MR63轴承座 + AS5600安装
// ============================================================

$fn  = 60;
TOL  = 0.15;
WALL = 4;      // 孔壁最小厚度 mm

// ---------- 管 ----------
TUBE_D   = 8;          // 铝管外径
TUBE_IN  = 16;         // 管插入深度

// ---------- 轴承 MR63 ----------
BEARING_OD = 6.0;      // 外径
BEARING_ID  = 3.0;     // 内径
BEARING_H   = 2.5;     // 厚度

// ---------- 摆杆轴 ----------
SHAFT_D = 3.0;         // 摆杆轴承段直径

// ---------- AS5600 ----------
AS5600_HOLE_SPAN = 12.5;  // 两安装孔中心距
AS5600_SCREW_D   = 2.0;   // M2 安装孔
AS5600_OFFSET    = 3.0;   // PCB 距轴承底面的间隙 (留给磁铁+芯片厚度)

// ---------- 顶丝 ----------
SCREW_D = 3.0;        // M3

// ============================================================
// 尺寸计算
// ============================================================
BOSS_R    = TUBE_D/2 + 4;           // 管套外半径
BODY_H    = TUBE_IN + 4;            // 管座总长 (水平)
BEARING_BOSS_H = 5;                 // 轴承座凸台高度

// 底部 AS5600 安装面总宽 (轴承座 + 两侧孔)
PCB_W = AS5600_HOLE_SPAN + 8;       // 比孔距宽一点

// ============================================================
module arm_end_mount() {
    difference() {
        union() {
            // ---- 水平管套 ----
            translate([0, 0, 0])
                rotate([0, 90, 0])
                    cylinder(d=TUBE_D + WALL*2, h=BODY_H);

            // ---- 过渡加强筋 (管套 → 轴承座) ----
            translate([BODY_H/2 - 2, -BOSS_R, -6])
                cube([4, BOSS_R*2, 6]);

            // ---- 轴承凸台 (向下) ----
            translate([BODY_H/2, 0, -BEARING_BOSS_H])
                cylinder(d=BEARING_OD + 4, h=BEARING_BOSS_H);

            // ---- AS5600 安装耳 (向下延伸) ----
            for (x = [-AS5600_HOLE_SPAN/2, AS5600_HOLE_SPAN/2]) {
                translate([BODY_H/2 + x, 0, -BEARING_BOSS_H])
                    cylinder(d=5, h=BEARING_BOSS_H + AS5600_OFFSET + 3);
            }
        }

        // ---- 铝管孔 ----
        translate([-1, 0, 0])
            rotate([0, 90, 0])
                cylinder(d=TUBE_D + TOL, h=BODY_H + 5);

        // ---- 摆杆轴通孔 (3mm) ----
        translate([BODY_H/2, 0, -BEARING_BOSS_H - AS5600_OFFSET - 5])
            cylinder(d=SHAFT_D + 0.3, h=BEARING_BOSS_H + AS5600_OFFSET + 15);

        // ---- 轴承座腔 (6mm OD, 从底部向上挖) ----
        translate([BODY_H/2, 0, -BEARING_BOSS_H])
            cylinder(d=BEARING_OD + 0.15, h=BEARING_H + 0.3);

        // ---- M2 AS5600 安装孔 ----
        for (x = [-AS5600_HOLE_SPAN/2, AS5600_HOLE_SPAN/2]) {
            translate([BODY_H/2 + x, 0, -BEARING_BOSS_H - AS5600_OFFSET - 5])
                cylinder(d=AS5600_SCREW_D + 0.2, h=10);
        }

        // ---- M3 顶丝 — 锁铝管 (从上面) ----
        translate([BODY_H/2, 0, BOSS_R])
            cylinder(d=SCREW_D + 0.2, h=10, center=true);
    }
}

arm_end_mount();


// ============================================================
// Assembly:
//   1. 8mm 铝管从左侧插入水平孔，顶丝锁紧
//   2. MR63 轴承从下方压入 6mm 腔
//   3. 摆杆轴 (3mm) 从上方穿过轴承
//   4. 径向磁铁粘在摆杆轴下端
//   5. AS5600 PCB 用 M2 螺丝固定在底部两耳
//   6. 磁铁距芯片面 ~1-2mm
// ============================================================
