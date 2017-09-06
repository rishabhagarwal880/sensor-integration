// "Flexible strain gauge"
// Alexi Charalambides
// April 2016

// 1) constants

// tool
var t = 2;
if (t == 1) {var r_tool = 0.2032;} // Microcut USA 82016 (mm)
if (t == 2) {var r_tool = 0.1016;} // Microcut USA 82008 (mm)
if (t == 3) {var r_tool = 0.0508;} // Microcut USA 82004 (mm)
if (t == 4) {var r_tool = 0.0254;} // Microcut USA 82002 (mm)
tool(t); // grab tool

// geometry
var gap = 0.020; // (mm)
var fingers = 6; // (mm)
var length = 2.5; // (mm)
var depth = 0.200; // (mm)

// 2) set initial mill parameters

var h_safe = 0.5; // safe height above workpiece (mm)
translate(0.0001, 0.0001, 0.0001); // translate X, Y, and Z to add decimal points to output file
rapid({z: h_safe}); // Move to a safe height
feed(6); // 6 mm/min is minimum feed rate of Roland
speed(10000); // Set RPM in the clockwise direction

// 3)  mill

rapid({x: 0, y: 0});
strain_gauge(gap, fingers, length, depth, r_tool, h_safe);

// 4) end milling

rapid({z: h_safe}); // Move back to safe height
speed(0); // Stop spinning










// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function strain_gauge(gap, fingers, length, depth, r_tool, h_safe) {

  var trace_length = 7; // (mm)

  // left trace
  icut({z: -h_safe-depth});
  icut({x: -trace_length});
  cpad();
  rapid({z: h_safe});
  irapid({x: trace_length});

  // left comb
  for (f=0; f<fingers; f++) {
    icut({z: -h_safe-depth});
    icut({y: 4*r_tool + 2*gap});
    icut({x: length});
    rapid({z: h_safe});
    irapid({x: -length});
  }

  // transition
  irapid({x: length + 2*r_tool + gap, y: 2*r_tool + gap});

  // right trace
  icut({z: -h_safe-depth});
  icut({x: trace_length});
  cpad();
  rapid({z: h_safe});
  irapid({x: -trace_length});

  // right comb
  for (f=0; f<fingers; f++) {
    icut({z: -h_safe-depth});
    icut({y: -(4*r_tool + 2*gap)});
    icut({x: -length});
    rapid({z: h_safe});
    irapid({x: length});
  }

}






// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cpad() {

  var length = 2; // length of square pad (mm)

  icut({y: length/2});
  icut({x: length/2});
  icut({y: -length});
  icut({x: -length});
  icut({y: length});
  icut({x: length/2});

  icut({y: -length/2});
  icut({x: length/2});
  icut({x: -length});
  icut({x: length/2});
  icut({y: -length/2});
  icut({y: length/2});

  // note: ends in the same place it started

}

