// ============================================================
// arm_mount.scad  —  Furuta 臂座
// 功能：连接 6mm 垂直主轴 → 8mm 水平铝管
// ============================================================

$fn  = 60;
TOL  = 0.15;   // 孔公差 (紧配合可减小)
WALL = 4;      // 孔壁最小厚度 mm

// ---------- 参数 ----------
SHAFT_D = 6;         // 主轴直径
TUBE_D  = 8;         // 铝管外径
TUBE_L  = 16;        // 管插入深度
SCREW_D = 3.0;       // 顶丝孔 (M3)

BODY_H  = 20;        // 总高度
BODY_W  = 18;        // 主轴块宽
BODY_D  = 18;        // 主轴块深
BOSS_D  = 16;        // 管套外径

// ---------- 主体 ----------
difference() {
    union() {
        // 主轴块 (居中)
        translate([-BODY_W/2, -BODY_D/2, -(TUBE_L/2+WALL+2)])
            cube([BODY_W, BODY_D, BODY_H]);

        // 管套 (水平伸出)
        translate([0, 0, -(TUBE_L/2+WALL) - TUBE_L/2])
            rotate([0, 90, 0])
                cylinder(d=BOSS_D, h=TUBE_L + WALL + 2);
    }

    // 主轴孔 6mm (垂直贯通)
    cylinder(d=SHAFT_D + TOL, h=BODY_H + 10, center=true);

    // 铝管孔 8mm (水平)
    translate([1, 0, -(TUBE_L/2+WALL) - TUBE_L/2])
        rotate([0, 90, 0])
            cylinder(d=TUBE_D + TOL, h=TUBE_L + WALL + 5);

    // M3 顶丝 — 锁主轴 (侧面 x)
    translate([0, 0, 3])
        rotate([0, 90, 0])
            cylinder(d=SCREW_D + 0.2, h=BODY_W/2 + 5);

    // M3 顶丝 — 锁铝管 (顶面)
    translate([TUBE_L/2 + 1, 0, -3])
        cylinder(d=SCREW_D + 0.2, h=10);
}


// ============================================================
// Usage:
//   1. 6mm 主轴从下方插入中心孔，侧面 M3 顶丝锁紧
//   2. 8mm 铝管从左侧插入水平孔，顶部 M3 顶丝锁紧
//   3. 臂座随主轴一起旋转，铝管径向伸出 = 水平臂
// ============================================================
