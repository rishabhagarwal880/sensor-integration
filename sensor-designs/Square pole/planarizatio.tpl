// 1) constants

// tool
r_tool = tool_change(1);


// set initial mill parameters
var h_safe = 0.5; // safe height above workpiece (mm)
translate(0.01, 0.01, 0.0001); // translate X, Y, and Z to add decimal points to output file
rapid({z: h_safe}); // Move to a safe height
feed(999.01); // 6 mm/min is minimum feed rate of Roland
speed(12000); // Set RPM in the clockwise direction

// milling Plane features

var dev_1 = 0;
var dev_2 = 38;

rapid({x:dev_2,y:-dev_2});
icut({z: -0.05});
for(i=-38; i<39;i=i++) plane(i);
rapid({z:h_safe});
speed(0);


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plane(pose) {
icut({x:-38});
icut({y:0.5});
icut({x:38});
icut({y:0.5});
//rapid({z:h_safe});
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tool_change(t) {
if (t == 1) {var r_tool = 0.2032;} // Microcut USA 82016 (mm)
if (t == 2) {var r_tool = 0.1016;} // Microcut USA 82008 (mm)
if (t == 3) {var r_tool = 0.0508;} // Microcut USA 82004 (mm)
if (t == 4) {var r_tool = 0.0254;} // Microcut USA 82002 (mm)
if (t == 5) {var r_tool = 0.3810/2; }//Microcut USA 72015 (mm)
tool(t); // grab tool
return r_tool
}