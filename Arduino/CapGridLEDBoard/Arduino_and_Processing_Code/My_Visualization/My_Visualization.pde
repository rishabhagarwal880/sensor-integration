import processing.serial.*; // add the serial library

Serial myPort; // define a serial port object to monitor
int padding = 100;

int screenX = 1024 ; // define screen dimensions
int screenY = 512;
int prevY;
float dT;
int currTime;
PrintWriter output;
boolean happen = false;

void setup() {
  output = createWriter("data_"+month()+"-"+day()+"-"+year()+"_"+hour()+"-"+minute()+".txt"); 

  size(screenX + 2*padding, screenY + 2*padding); // set the window size
  background(200, 200, 200);
  println(Serial.list()); // list all available serial ports
  myPort = new Serial(this, Serial.list()[1], 9600); // define input port
  myPort.clear(); // clear the port of any initial junk
  fill(255, 255, 255); // pick the fill color (r,g,b)
  stroke(0, 0, 0);
  rect(padding, padding, screenX, screenY);
  dT = .1;
  prevY = 0;
  currTime = padding + 1;
  textAlign(CENTER, TOP);
  stroke(0, 0, 0);
  textSize(24);
  text("Capacitance", 512 + padding, padding/2);
  // Draw Voltage Tick marks
  for (int i = screenY; i >= 0; i --) {
    if (i % (screenY/2) == 0) {
      line(padding - 20, i + padding, padding, i + padding);
    }
    else if (i % (screenY/4) == 0) {
      line(padding - 15, i + padding, padding, i + padding);
    }
    else if (i % 16 == 0) {
      line(padding - 5, i + padding, padding, i + padding);
    }
  }
  textSize(12);
  text("4.096 pF", padding/2, padding - 5);
  text("0 pF", padding/2, padding + screenY/2 - 5);
  text("-4.096 pF", padding/2, padding + screenY - 5);
  // Draw Time tick marks
  for (int j = screenX; j >= 0; j --) {
    if (j % 16 == 0) {
      line(j + padding, screenY + padding, j + padding, screenY + padding + 10);
    }
  }
}
void draw () {
  float prevCapVal = 0; 
  float prevCapVal2 = 0; 
  int ScreenYPosition = 0;
  int ScreenYPosition2 = 0;
  int ScreenYPosition3 = 0;
  int ScreenYPosition4 = 0;


  while (myPort.available () > 0) { // make sure port is open
    String inString = myPort.readStringUntil('\n'); // read input string
    if (inString != null) { // ignore null strings
      inString = trim(inString); // trim off any whitespace
      String[] tokenized = splitTokens(inString, ","); // extract x & y into an array
      // proceed only if correct # of values extracted from the string:
      if (tokenized.length == 4) {

        println(inString);
        float capVal = (1.0 * int(tokenized[0])) / 1000.0;
        float capVal2 = (1.0 * int(tokenized[1])) / 1000.0;
        float capVal3 = (1.0 * int(tokenized[2])) / 1000.0;
        float capVal4 = (1.0 * int(tokenized[3])) / 1000.0;


        if (!happen) {
          prevCapVal = capVal;
          prevCapVal2 = capVal2;
          ScreenYPosition = int(screenY - (capVal)/(2*4.096)*(screenY));
          ScreenYPosition2 = int(screenY - (capVal2)/(2*4.096)*(screenY));
          ScreenYPosition3 = int(screenY - (capVal3)/(2*4.096)*(screenY));
          ScreenYPosition4 = int(screenY - (capVal4)/(2*4.096)*(screenY));

          happen =true;
        }
        else {
          // map values to graph
          println(capVal-4.096);
          int scale = 50;
          ScreenYPosition = int(screenY - (capVal)/(2*4.096)*(screenY));
          ScreenYPosition2 = int(screenY - (capVal2)/(2*4.096)*(screenY));
          ScreenYPosition3 = int(screenY - (capVal3)/(2*4.096)*(screenY));
          ScreenYPosition4 = int(screenY - (capVal4)/(2*4.096)*(screenY));

          //        ScreenYPosition = ScreenYPosition+int(scale*(capVal-prevCapVal));
          //        ScreenYPosition2 = ScreenYPosition2 + int(scale*(capVal2-prevCapVal2));

          println(ScreenYPosition);
          stroke(255, 255, 255);
          line(currTime, padding + 1, currTime, screenY + padding - 1);

          fill(0, 0, 255); // pick the fill color (r,g,b)
          stroke(0, 0, 255);
          ellipse(currTime, ScreenYPosition + padding, 3, 3);

          fill(255, 0, 0); // pick the fill color (r,g,b)
          stroke(255, 0, 0);
          ellipse(currTime, ScreenYPosition2 + padding, 3, 3);

          fill(0, 255, 0); // pick the fill color (r,g,b)
          stroke(0, 255, 0);
          ellipse(currTime, ScreenYPosition3 + padding, 3, 3);
          
          fill(0, 255, 255); // pick the fill color (r,g,b)
          stroke(0, 255, 255);
          ellipse(currTime, ScreenYPosition4 + padding, 3, 3);

          
          prevCapVal = capVal;
          prevCapVal2 = capVal2;
        }


        // Print to file
        output.println(inString);
        output.flush();  // flushing everytime is not efficient, need something that catches when the sketch is closing and flushes and closes the file then

        // Increment x location            
        currTime += dT * 10;
        if (currTime % (screenX  + padding - 1) == 0) {
          currTime = padding + 1;
        }
      }
    }
  }
}

public void stop() {
  myPort.stop();
  output.close();
  super.stop();
} 

