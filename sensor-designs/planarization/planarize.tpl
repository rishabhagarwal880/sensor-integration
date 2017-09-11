// Initial mill parameters

var h_safe = 0.5; // safe height above workpiece (mm)
translate(0.0001, 0.0001, 0.0001); // translate X, Y, and Z to add decimal points to output file
rapid({z: h_safe}); // Move to a safe height
feed(6.01); // 6 mm/min is minimum feed rate of Roland
speed(12000); // Set RPM in the clockwise direction


// 3) Planarizing
tool(5);
r_tool = tool_radius(5);
var move_p = 999.01 // Feed rate of planarization
//var h_move = 999.01;   // Feed rate of horizontal move
//var v_move = 10.01;    // Feed rate of vertical move
var depth_p = 0.25;   // Depth of planarization
var xlength = 70;  // Cutting length in x-direction
var ylength = 70; // Cutting length in y-direction
var size = ylength; // variable for sizing (the unit should be in mm)
planar(size,xlength,ylength,depth_p,r_tool,h_safe,move_p)


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function planar(size,xlength,ylength,depth,r_tool,h_safe,move_p){
feed(move_p);
speed(12000);
rapid({x:-xlength/2,y:ylength/2}); 
for (i=1; i<=size; i++){

    cut({z:-depth});
    //feed(h_move);
    icut({x: xlength});
    //feed(v_move);
    icut({y: -0.5});
    //feed(h_move);
    icut({x:-xlength});
    //feed(v_move);
    icut({y: -0.5});
    //feed(h_move);
  }  
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tool_radius(t) {

if (t == 1) {var r_tool = 0.2032;} // Microcut USA 82016 (mm)
if (t == 2) {var r_tool = 0.1016;} // Microcut USA 82008 (mm)
if (t == 3) {var r_tool = 0.0508;} // Microcut USA 82004 (mm)
if (t == 4) {var r_tool = 0.0254;} // Microcut USA 82002 (mm)
if (t == 5) {var r_tool = 3.1750;} // 0.25 square

return r_tool

}