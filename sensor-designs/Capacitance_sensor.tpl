// 0) number of duplications (in the array)

var c = 2; // number of columns
var r = 2; // number of rows
var w = 40; // distances between columns (widths)
var h = 15; // distance between rows (heights)

// 1) set constants

// sensor
var gap = 0.2; // electrode gap (mm)
var wedge_w = 1.200; // wedge width (mm)
var wedge_l = 1.200; // wedge length (mm)
var wedge_h = 0.500; // wedge height (mm)
var pillar_h = 1.500; // pillar height (mm) -- seems to melt gaps at deep depths. maybe mill pillar first?

// traces
var trace_h = 0.250; // trace height (mm)
var trace_l = 20; // trace length (mm)

// tools
var t_pad = 1; // 400 um diameter
var t_pillar = 5; // 400 um diameter
var t_connector = 3; // 100 um diameter

// 2) set initial mill parameters

feed(6.01);
speed(10000);
translate(0.0001,0.0001, 0.0001); // add 4 decimals to gcode output
var h_safe = 1; // safe height above workpiece (mm)
rapid({z: h_safe}); // Move to a safe height
rapid({x: 0, y: 0}); // Go to start position

// 3) mill

// initialize parameters for array positionining

var ww = (c-1)*w; // total width all columns oppcupy (neglecting the width of the sensors themselves)
var hh = (r-1)*h; // total height all rows oppcupy (neglecting the height of the sensors themselves)

var x0 = 0-ww/2; // x-coordinate of starting point in order to center the array (each column is at x0 + (x-1)*w)
var y0 = 0-hh/2; // y-coordinate of starting point in order to center the array (each row is at y0 + (y-1)*h)

var x = 0;
var y = 0;

// pillar
tool(t_pillar);
for (x = 0; x <c; x = x + 1) {
for (y = 0; y <r; y = y + 1){ 
rapid({z: h_safe});
pillar(x0 + x*w, y0 + y*h, pillar_h, h_safe);
}
}

// pads
tool(t_pad);
for (x = 0; x <c; x = x + 1) {
 for (y = 0; y <r; y = y + 1){ 
rapid({z: h_safe});
pads(x0 + x*w, y0 + y*h, gap, wedge_w, wedge_l, wedge_h, t_pad, t_pillar, h_safe);
}
}

// traces
tool(t_pillar);
for (x = 0; x <c; x = x + 1) {
 for (y = 0; y <r; y = y + 1){ 
rapid({z: h_safe});
traces(x0 + x*w, y0 + y*h, gap, trace_h, trace_l, t_pad, t_pillar, h_safe);
}
}

// pillar-trace connector
tool(t_connector);
for (x = 0; x <c; x = x + 1) {
for (y = 0; y <r; y = y + 1){ 
rapid({z: h_safe});
connector(x0 + x*w, y0 + y*h, trace_h, h_safe);
}
}





// 4) end milling

rapid({z: h_safe}); // Move back to safe height















// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pads(pos_x, pos_y, gap, wedge_w, wedge_l, wedge_h, t_pad, t_pillar, h_safe) {

  var offset = tool_radius(t_pillar) + gap + tool_radius(t_pad);

//  triangle(pos_x-offset, pos_y, wedge_w, wedge_l, wedge_h, 4, tool_radius(t_pad), h_safe); // left
  square(pos_x, pos_y+offset, wedge_w, wedge_l, wedge_h, 1, tool_radius(t_pad), h_safe); // top
  square(pos_x+offset, pos_y, wedge_w, wedge_l, wedge_h, 2, tool_radius(t_pad), h_safe); // right
  square(pos_x, pos_y-offset, wedge_w, wedge_l, wedge_h, 3, tool_radius(t_pad), h_safe); // bottom

}





// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function square(pos_x, pos_y, w, l, h, orientation, r_tool, h_safe) {

  rapid({z: h_safe});
  rapid({x: pos_x, y: pos_y});
  icut({z: -h_safe-h});

  if (orientation == 1) {    // cut towards back of milling machine
    icut({y:l/4});
    icut({x: w/4});
    icut({y: -l/4});
    icut({x: -w/4});
  }
  if (orientation == 2) {    // cut towards right of milling machine
    icut({y: -w/4});
    icut({x: l/4});
    icut({y: w/4});
    icut({x: -l/4});
  }
  if (orientation == 3) {    // cut towards front of milling machine
    icut({y: -l/4});
    icut({x: -w/4});
    icut({y: l/4});
    icut({x: w/4});
  }
//  if (orientation == 4) {    // cut towards left of milling machine
//    icut({x: -l, y: w/2});
//    icut({y: -w});
//    icut({x: l, y: w/2});
//  }

  // call recursively (doesn't work when wedge_w >> wedge_l)
  if (w > 2*r_tool && l > 2*r_tool) {

    if (orientation == 1) {var x_dir = 0; var y_dir = 1;}
    if (orientation == 2) {var x_dir = 1; var y_dir = 0;}
    if (orientation == 3) {var x_dir = 0; var y_dir = -1;}
    if (orientation == 4) {var x_dir = -1; var y_dir = 0;}

    square(pos_x+2*r_tool*x_dir, pos_y+2*r_tool*y_dir, w-3*r_tool, l-3*r_tool, h, orientation, r_tool, h_safe);
  }

}





// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pillar(pos_x, pos_y, pillar_h, h_safe) {

  rapid({z: h_safe});
  rapid({x: pos_x, y: pos_y});
  icut({z: -h_safe-pillar_h});

}





// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function connector(pos_x, pos_y, trace_h, h_safe) {

  rapid({z: h_safe});
  rapid({x: pos_x, y: pos_y});
  icut({z: -h_safe-trace_h});
  icut({x: -1, y: -1});

}







// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function traces(pos_x, pos_y, gap, trace_h, trace_l, t_pad, t_pillar, h_safe) {

  var K = 0.250; // factor of safety distance from pillar (mm)
  var offset = tool_radius(t_pillar) + gap + tool_radius(t_pad) + K;

  // left
 // rapid({z: h_safe});
 // rapid({x: pos_x-offset, y: pos_y});
 // icut({z: -h_safe-trace_h});
 // icut({x: -1.5});
 // icut({y: 2.05});
 // icut({x: 2.05, y: 2.05});
 // icut({x: trace_l-2.05});
//  ipad()

  // top
  rapid({z: h_safe});
  rapid({x: pos_x, y: pos_y+offset});
  icut({z: -h_safe-trace_h});
  icut({y: 1.5});
  icut({x: trace_l+1});
  ipad()

  // right
  rapid({z: h_safe});
  rapid({x: pos_x+offset, y: pos_y});
  icut({z: -h_safe-trace_h});
  icut({x: trace_l+3.5});
  ipad()

  // bottom
  rapid({z: h_safe});
  rapid({x: pos_x, y: pos_y-offset});
  icut({z: -h_safe-trace_h});
  icut({y: -1.5});
  icut({x: trace_l+7.3});
  ipad()

  // pillar
  rapid({z: h_safe});
  rapid({x: pos_x-1, y:  pos_y-1});
  icut({z: -h_safe-trace_h});
  icut({x: -1.1, y: -1.1});
  icut({y: -2});
  icut({x: trace_l+12.5});
  ipad()

}








// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ipad() {

  var length = 2; // length of square pad (mm)

  icut({x: length/2});
  icut({y: length});
  icut({x: -length});
  icut({y: -length});
  icut({y: length/2});
  icut({x: length});
  icut({x: -length/2});
  icut({y: -length/2});
  icut({y: length});

}







// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tool_radius(t) {

if (t == 1) {var r_tool = 0.2032;} // Microcut USA 82016
if (t == 2) {var r_tool = 0.1016;} // Microcut USA 82008
if (t == 3) {var r_tool = 0.0508;} // Microcut USA 82004
if (t == 4) {var r_tool = 0.0254;} // Microcut USA 82002 
if (t == 5) {var r_tool = 0.1905;} // Microcut USA 72015 (long reach)

return r_tool

}