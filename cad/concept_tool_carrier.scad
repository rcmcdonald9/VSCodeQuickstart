$fn=48;
module rbox(s, r=8){ hull() for(x=[r,s[0]-r],y=[r,s[1]-r],z=[r,s[2]-r]) translate([x,y,z]) sphere(r); }
// hoverboard wheels (axle along Y)
for(s=[-1,1]) translate([0,s*285,85]) rotate([90,0,0]) {
  color("#222") cylinder(d=170,h=62,center=true);
  color("#888") cylinder(d=110,h=64,center=true);
}
// hoverboard body
color("#3a3d44") translate([-90,-255,45]) rbox([180,510,85],20);
// printed chassis / deck
color("#e9e6df") translate([-280,-200,125]) rbox([420,400,70],25);
// rear caster
color("#444") translate([-250,0,0]) { translate([0,0,40]) sphere(d=70); translate([-15,-20,60]) cube([30,40,70]); }
// battery/electronics hump
color("#f2a03d") translate([-240,-150,190]) rbox([260,300,55],25);
// mast + head
color("#cfcfcf") translate([-160,0,240]) cylinder(d=36,h=230);
translate([-160,0,470]) {
  color("#2f6fd6") translate([-50,-65,0]) rbox([130,130,105],28);
  color("#111") for(y=[-28,28]) translate([80,y,58]) rotate([0,90,0]) cylinder(d=38,h=4);
  color("#5ff") for(y=[-28,28]) translate([84,y,58]) rotate([0,90,0]) cylinder(d=18,h=2);
  color("#2f6fd6") for(y=[-42,42]) translate([0,y,95]) hull(){ translate([-20,-6,0]) cube([40,12,1]); translate([-5,-4,48]) cube([10,8,1]); }
  color("#222") translate([60,-10,15]) cube([25,20,14]);
}
// lift arms + quick-attach plate
color("#f2a03d") for(s=[-1,1]) hull(){ translate([60,s*180,160]) rotate([90,0,0]) cylinder(d=40,h=20,center=true); translate([250,s*180,90]) rotate([90,0,0]) cylinder(d=34,h=20,center=true); }
color("#555") for(s=[-1,1]) hull(){ translate([40,s*150,150]) sphere(10); translate([200,s*150,120]) sphere(10); }
color("#e9e6df") translate([250,-210,40]) cube([14,420,130]);
// bucket
color("#2f6fd6") translate([264,0,0]) difference(){
  hull(){ translate([0,-250,10]) cube([10,500,150]); translate([150,-250,0]) cube([10,500,20]); translate([0,-250,0]) cube([10,500,10]); }
  hull(){ translate([8,-242,18]) cube([10,484,150]); translate([150,-242,14]) cube([20,484,20]); }
}
// rake bar on top of bucket
color("#888") translate([300,-240,175]) rotate([0,90,0]) rotate([0,0,0]) cylinder(d=12,h=10);
color("#888") translate([280,0,200]) rotate([90,0,0]) cylinder(d=14,h=480,center=true);
color("#666") for(y=[-220:40:220]) translate([280,y,200]) rotate([0,135,0]) cylinder(d=6,h=90);
// toys in bucket
for(p=[[330,-120,30,"#e33"],[360,40,32,"#3c3"],[320,150,28,"#fc2"]]) color(p[3]) translate([p[0],p[1],p[2]]) cube(40,center=true);
color("#e33") translate([480,-60,25]) sphere(25);
color("#39f") translate([520,120,22]) cube(44,center=true);
// rear hitch + towed roller picker
color("#555") hull(){ translate([-280,0,110]) sphere(12); translate([-520,0,95]) sphere(12); }
translate([-560,0,90]) {
  color("#777") rotate([90,0,0]) cylinder(d=10,h=600,center=true);
  color("#f2a03d") for(y=[-280:22:280]) translate([0,y,0]) rotate([90,0,0]) difference(){ cylinder(d=180,h=8,center=true); cylinder(d=150,h=10,center=true);}
  color("#e9e6df") translate([-60,-300,60]) cube([120,600,8]);
}
for(p=[[-700,-150],[-680,60],[-740,200],[-660,-260]]) color("white") translate([p[0],p[1],21]) sphere(21);
// floor
color("#cfd8c8") translate([-900,-700,-2]) cube([1700,1400,2]);
