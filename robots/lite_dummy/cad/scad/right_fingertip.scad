% scale(1000) import("right_fingertip.stl");
% scale(1000) import("right_wrist_pitch.stl");

translate([704, -7, 22])
rotate([0, -90, 0])
cylinder(r=6, h=60);

translate([580, 10, 0])
rotate([0, -90, 0])
cylinder(r=35, h=40, center=true);
