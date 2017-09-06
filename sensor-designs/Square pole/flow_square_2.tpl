// "flow sensor"
// Rishabh
// Feb. 2017

// 1) constants

// tool
r_tool = tool_change(1);

// 1) Sensor features geometry
var gap_1 = 0.025; // (mm)
var gap_2 = 0.030; // (mm)
var gap_3 = 0.035; // (mm)

var length = 0.4; // (mm) mid pad length

var depth_s =  0.15; //(mm) pad area
var depth_p = 1.2; // post

// 3) set initial mill parameters
var h_safe = 0.5; // safe height above workpiece (mm)
translate(0.0001, 0.0001, 0.0001); // translate X, Y, and Z to add decimal points to output file
rapid({z: h_safe}); // Move to a safe height
feed(6.01); // 6 mm/min is minimum feed rate of Roland
speed(12000); // Set RPM in the clockwise direction


// 4) milling shear sensor features

var dev_1 = 0;
var dev_2 = 15;

// i) traces
//bottom
rapid({x:dev_1,y:-dev_2});
tracesp(gap_1,depth_s, length,h_safe);
rapid({x:dev_1,y:-dev_2});
tracemp(gap_1,depth_s, length,h_safe);
//mid
rapid({x:dev_1,y:dev_1});
tracesp(gap_2,depth_s, length,h_safe);
rapid({x:dev_1,y:dev_1});
tracemp(gap_2,depth_s, length,h_safe);
//top
rapid({x:dev_1,y:dev_2});
tracesp(gap_3,depth_s, length,h_safe);
rapid({x:dev_1,y:dev_2});
tracemp(gap_3,depth_s, length,h_safe);

r_tool = tool_change(2);

// ii) Surrounding pads
// bottom
rapid({x:dev_1,y:-dev_2});
surroundpad(gap_1,depth_s, length,h_safe);
// mid
rapid({x:dev_1,y:dev_1});
surroundpad(gap_2,depth_s, length,h_safe);
//top
rapid({x:dev_1,y:dev_2});
surroundpad(gap_3,depth_s, length,h_safe);

// iii) mid pads
r_tool = tool_change(5);

// bottom
rapid({x:dev_1,y:-dev_2});
midpad(gap_1,depth_p, length,h_safe);
// mid
rapid({x:dev_1,y:dev_1});
midpad(gap_2,depth_p, length,h_safe);
//top
rapid({x:dev_1,y:dev_2});
midpad(gap_3,depth_p, length,h_safe);


//iV)Post
//r_tool = tool_change(1);

//mid
//rapid({x:dev_1,y:dev_1});
//icut({z:-h_safe-depth_p});
//rapid({z:h_safe});

//top
//rapid({x:dev_1,y:dev_2});
//icut({z:-h_safe-depth_p});
//rapid({z:h_safe});

//bottom
//rapid({x:dev_1,y:-dev_2});
//icut({z:-h_safe-depth_p});
//rapid({z:h_safe});



// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function midpad(gap, depth,length, h_safe) {
// mid pad
 icut({z: -h_safe-depth});
icut({x:-length/4});
icut({x:length/2});
icut({x:-length/4});
icut({y:-length/4});
 icut({y:length/2});
 icut({x:-length/4});
 icut({y:-length/2});
 icut({x:length/2});
icut({y:length/2});
icut({x:-length/4});
rapid({z:h_safe});
}

function surroundpad(gap, depth, length,h_safe) {
// surrounding pads (clockwise from top)
//pad 1
irapid({x:-length/2,y:gap+2*r_tool+length/2});
 icut({z: -h_safe-depth});
 icut({x:length});
 icut({x:-length/2});
 icut({y:2*length});
 icut({y:-2*length});
icut({x:length/2});
 icut({y:2*length});
 icut({x:-length});
 icut({y:-2*length});
 icut({x:length})
rapid({z:h_safe});

//pad 2
irapid({x:gap+2*r_tool,y:-(gap+2*r_tool)});
 icut({z: -h_safe-depth});
 icut({y:-length});
 icut({y:length/2});
 icut({x:2*length});
 icut({x:-2*length});
icut({y:-length/2});
 icut({x:2*length})
 icut({y:length}) 
 icut({x:-2*length}) 
 icut({y:-length})
rapid({z:h_safe});

//pad 3
irapid({x:-(gap+2*r_tool),y:-(gap+2*r_tool)});
 icut({z: -h_safe-depth});
 icut({x:-length});
 icut({x:length/2});
 icut({y:-2*length});
 icut({y:2*length});
icut({x:-length/2});
 icut({y:-2*length})
 icut({x:length})
 icut({y:2*length})
 icut({x:-length})
rapid({z:h_safe});
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tracesp(gap,depth, h_safe,type) {
//pad 1
irapid({y:length/2+gap+2*r_tool});
 icut({z: -h_safe-depth});
 icut({y:5});
icut({x:20});
cpad(1.5);
rapid({z:h_safe});

//pad 2
irapid({y:-5-length/2-gap-2*r_tool});
 icut({z: -h_safe-depth});
 cpad(1.5);
 icut({x: -20+length/2+gap+2*r_tool});
rapid({z:h_safe});

//pad 3
irapid({x:-length/2-gap-2*r_tool,y:-length/2-gap-2*r_tool});
 icut({z: -h_safe-depth});
 icut({y:-5});
icut({x:20});
cpad(1.5);
rapid({z:h_safe});
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tracemp(gap,depth, h_safe,type) {
//pad 1
irapid({x:-length/2});
 icut({z: -h_safe-depth});
 icut({x:-20});
cpad(1.5);
rapid({z:h_safe});

}
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cpad(length) {
//connectors
  //length =  length of square pad (mm)

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


