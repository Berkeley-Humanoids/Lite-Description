% scale(1000) import("torso.stl");

translate([0, 0, 350])
cylinder(r=40, h=120, center=true);

translate([0, 0, 380])
rotate([0, 90, 0])
cylinder(r=50, h=80, center=true);
