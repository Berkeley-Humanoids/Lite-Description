% scale(1000) import("chest.stl");

translate([0, 25, 180])
rotate([13, 0, 0])
cylinder(r=53, h=160, center=true);

translate([40, 15, 220])
rotate([13, -10, 0])
cube([40, 80, 180], center=true);

translate([-40, 15, 220])
rotate([13, 10, 0])
cube([40, 80, 180], center=true);
