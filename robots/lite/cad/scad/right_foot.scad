% scale(1000) import("right_foot.stl");

translate([70, -10, -760])
cube([65, 120, 30], center=true);

translate([70, 85, -750])
rotate([20, 0, 0])
cube([70, 50, 30], center=true);
