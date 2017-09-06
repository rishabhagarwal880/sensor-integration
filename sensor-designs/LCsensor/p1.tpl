function square(size, depth, safe) {
  cut({z: depth});  // Cut down to depth
  icut({x: size});  // Cut to second corner
  icut({y: size});  // Cut to third corner
  icut({x: -size}); // Cut to forth corner
  icut({y: -size}); // Cut back to begining
  rapid({z: safe}); // Move back to safe position
}

feed(400);          // Set the feed rate to 400 mm per minute
tool(1);            // Select tool 1

rapid({z: 5});      // Move to a safe height of 5mm
rapid({x: 1, y: 1});  // Go to start position
speed(2000);        // Spin at 2000 RPM in the clockwise direction

square(10, -3, 5);

speed(0);           // Stop spinning
