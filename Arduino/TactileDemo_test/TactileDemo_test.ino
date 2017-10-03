#include <Wire.h>

//#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define PIN 6

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

// AD7746 
#define I2C_ADDRESS 0x48        // From AD7746_clean.ino. It is the Write or read Addr shift 1 bit to 
                                // the right thus this is the 7-bit address part of the start byte
                                // the arduino Wire library should take care of the read/write bit
#define WRITE_ADDRESS 0x90
#define READ_ADDRESS 0x91
#define RESET_ADDRESS 0xBF

// Register Definitions
#define REGISTER_STATUS 0x00     // Read-only, Indicates status of the converter
#define REGISTER_CAP_DATA 0x01   // 24 bits includes 0x02 and 0x03
#define REGISTER_VT_DATA 0x04    // 24 bits includes 0x05 and 0x06
#define REGISTER_CAP_SETUP 0x07  // Capacitive channel setup
#define REGISTER_VT_SETUP 0x08   // Voltage/Temperature channel setup
#define REGISTER_EXC_SETUP 0x09  // Capacitive channel excitation setup
#define REGISTER_CONFIG 0x0A     // Converter update rate and mode of operation setup
#define REGISTER_CAP_DAC_A 0x0B  // Capacitive DAC setup
#define REGISTER_CAP_DAC_B 0x0C  // Capacitive DAC setup
#define REGISTER_CAP_OFFSET 0x0D // 16 bits includes register 0x0E
#define REGISTER_CAP_GAIN 0x0F   // 16 bits includes register 0x10
#define REGISTER_VOLT_GAIN 0x11  // 16 bits includes register 0x12

// --- Mode Definitions ---
// CAP_SETUP Modes
#define CAPEN _BV(7)             // Enables capacitive channel for single or continuous conversion or calibration
#define CIN2 _BV(6)              // Switches the internal multiplexer to the second capacitive input 
#define CAPDIFF _BV(5)           // Sets differntial mode on the selected capacitive input
#define CAPCHOP _BV(0)           // Doubles capacitive channel conversion times

// EXC_SETUP Modes
#define CLKCTRL _BV(7)           // Decreases the excitation signal freq and the clock freq by a factor of 2
#define EXCON _BV(6)             // Exc. signal is present on the output during both cap and vt conversion
#define EXCB _BV(5)              // Enables EXCB pin as the excitation output
#define NOTEXCB _BV(4)           // Enables EXCB pin as the inverted excitation output
#define EXCA _BV(3)              // Enables EXCA pin as the excitation output
#define NOTEXCA _BV(2)           // Enables EXCA pin as the inverted excitation output
// Excitation Voltage Level Modes
#define EXCVL_LOWEST 0                // Voltage on Cap +-Vdd/4 
#define EXCVL_LOW _BV(0)           // Voltage on Cap +-Vdd/4 
#define EXCVL_HIGH _BV(1)           // Voltage on Cap +-Vdd * 3/8
#define EXCVL_HIGHEST _BV(0) | _BV(1)  // Voltage on Cap +-Vdd/2

// CONFIG Modes
// Capacitive channel digital filter setup -- conversion time/update rate setup
#define CAPF_FASTEST 0                         // Conversion Time: 11.0ms, Update Rate: 90.9, Hz: 87.2
#define CAPF_FASTER _BV(0)                     // Conversion Time: 11.9ms, Update Rate: 83.8, Hz: 79.0
#define CAPF_FAST _BV(1)                       // Conversion Time: 20.0ms, Update Rate: 50.0, Hz: 43.6
#define CAPF_MEDFAST _BV(0) | _BV(1)           // Conversion Time: 38.0ms, Update Rate: 26.3, Hz: 21.8
#define CAPF_MEDSLOW _BV(2)                    // Conversion Time: 62.0ms, Update Rate: 16.1, Hz: 13.1
#define CAPF_SLOW _BV(0) | _BV(2)              // Conversion Time: 77.0ms, Update Rate: 13.0, Hz: 10.5
#define CAPF_SLOWER _BV(1) | _BV(2)            // Conversion Time: 92.0ms, Update Rate: 10.9, Hz: 8.9
#define CAPF_SLOWEST _BV(0) | _BV(1) | _BV(2)  // Conversion Time: 109.6ms, Update Rate: 9.1, Hz: 8.0
// Converter Mode of operation setup
#define MD_IDLE 0                           // Idle
#define MD_CONTINOUS _BV(0)                      // Continuous
#define MD_SINGLE _BV(1)                      // Single conversion
#define MD_POWERDOWN _BV(0) | _BV(1)             // Power-Down
#define MD_CAPOFFSETCAL _BV(0) | _BV(2)             // Capacitive system offset calibration
#define MD_CAPVTGAINCAL _BV(0) | _BV(1) | _BV(2)    // Capacitance or voltage system gain calibration

// Calibration Variables
#define VALUE_UPPER_BOUND 0xE00000UL                  // Maximum pF value of approximately 3pF
#define VALUE_LOWER_BOUND 0x800000UL                  // Minimum pF value of 0
#define MAX_OUT_OF_RANGE_COUNT 3
#define CALIBRATION_INCREASE 1

const int numSensor = 2;
int sensors[] = {2, 5};  // The sensor at index 0 is the most sensitive

byte calibration[numSensor];
float sensorRange = 300;
float scale = 1;

// --- Multiplexer Variables ---
int TOP_S0_PIN = 8;
int TOP_S1_PIN = 9;
int TOP_S2_PIN = 10;

int BOT_S0_PIN = 3;
int BOT_S1_PIN = 4;
int BOT_S2_PIN = 5;

// --- Sensor Flags ---


// Status Register shortcut
int CAPINT = 2;  // This pin is connected to the status register and a falling edge (going from
                 // high to low) indicates a conversion is finished.
int prevState;
int currState;

unsigned long offset = 0;   // This is a temporary measure it should be found during the calibration step.
int ind = 0;
String transmit = "";
void setup()
{
  pinMode(7,OUTPUT);
//  strip.begin();
//  for(uint16_t i=0; i<strip.numPixels(); i++) {
//      strip.setBrightness(10);  // prev 10
//  }
//  strip.show(); // Initialize all pixels to 'off'
  
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW);
  
   // --- Configure Multiplexer ---
  pinMode(TOP_S0_PIN,OUTPUT);
  pinMode(TOP_S1_PIN,OUTPUT);
  pinMode(TOP_S2_PIN,OUTPUT);
  pinMode(BOT_S0_PIN,OUTPUT);
  pinMode(BOT_S1_PIN,OUTPUT);  
  pinMode(BOT_S2_PIN,OUTPUT);

  Wire.begin();                 // Set up I2C for operation
  Serial.begin(9600);           // Set Baud Rate
  
  delay(15);
  Serial.println("Setup Begin");
  
  //  --- Reset Device ---
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(RESET_ADDRESS);    // reset device
  Wire.endTransmission();
  
  delay(1);                     // need to wait at least 200 microseconds for device to reset

  // --- Configure Modes ---
  writeConfig(REGISTER_CAP_SETUP, _BV(7));
  writeConfig(REGISTER_EXC_SETUP, _BV(3));
  
  
  // wait for calibration
  delay(1000);
  
  
  writeConfig(REGISTER_CONFIG, _BV(7) | _BV(6) /*| _BV(5)*/ | _BV(4) | _BV(3)  | _BV(0)); // slowest with cont conversion
  delay(5000);

  // --- Configure CAPINT and initalize loop variables---
  pinMode(CAPINT,INPUT);
  prevState = digitalRead(CAPINT);
  
  Serial.println("Setup End");
  for(int i = 0; i < numSensor; i++){
      delay(2000);
      selectCapacitor(sensors[i]);
      readValue();
      delay(2000);
      readValue();            
      delay(1000);      
      calibrate(i);
  }

}

void loop()
{
  unsigned long codes[numSensor];

  for(int i = 0; i< numSensor; i++){

    selectCapacitor(sensors[i]);  
    writeConfig(REGISTER_CAP_DAC_A, _BV(7) | calibration[i]);
    delay(10);

    currState = digitalRead(CAPINT);
    for (int j = 0; j < 1; j++){
      // if a new conversion has finished NOTE: Look up conversion rate and
      // see how much the delays will affect it
      while(!(prevState == 1 && currState == 0)){   
        prevState = currState;
        delay(1);
        currState = digitalRead(CAPINT);
      }
    delay(10);
    readValue();            
    delay(10);
      
    codes[i] = readValue();
    }
  } 
  
    
  String outString = String(int(1000*(4.096*2.0*(codes[0]))/(1.0 * 0xffffffUL)-4.096)) + "," + 
                     String(int(1000*(4.096*2.0*(codes[1]))/(1.0 * 0xffffffUL)-4.096));
  Serial.println(outString);
    
  //updateLEDs(strip.Color(0, 255, 0), 50,codes); // Update LED Array
}
//// Fill the dots one after the other with a color
//void updateLEDs(uint32_t c, uint8_t wait,unsigned long code[]) {
//  for(int j = 0; j < numSensor; j++){
//    float val = 1000*(4.096*2*(code[j]))/(1.0 * 0xffffffUL);
//
//     int range = int((normalizationVal[j]-val)/(sensorRanges[j])*255);
//     if(range > 255){  
//       range = 255; 
//     }
//     if(range < 0){  
//       range = 0; 
//     }
//     uint32_t newC;
//     if(range >= light_threshold){
//       newC = strip.Color(255, 0, 0);
//     }else{
//       newC = c; 
//     }
//     strip.setPixelColor(2*j + ((j >= 2)? 4 : 0), newC);
//     strip.setPixelColor(2*j+ 1 + ((j >= 2)? 4 : 0), newC);
//     strip.setPixelColor(2*j + 4 + ((j >= 2)? 4 : 0), newC);
//     strip.setPixelColor(2*j + 5 + ((j >= 2)? 4 : 0), newC);
//     strip.show();
//  }
//  
//}

/*
   Writes a bitmask to the specified address.  This configures
   the operation of the AD7746 
*/
void writeConfig(unsigned char address, unsigned char mask)
{
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(address);  // may have to resolve to two bytes
    Wire.write(mask);
    Wire.endTransmission();
    delay(1);
}
/*
  Reads numBytes bytes from the selected address and adds
  the bytes into one number. 
  NOTE: May want to remove Signaling printlines for normal operation
  NOTE: Add error checking for numBytes, i.e. less than 1
*/
unsigned long readRegister(unsigned char address, unsigned int numBytes)
{ 
  union {
    char data[3];
    unsigned long code;
  } byteMappedCode;
  
  byteMappedCode.code = 0;
  
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  //Wire.requestFrom(READ_ADDRESS, numBytes);
  Wire.requestFrom(I2C_ADDRESS, numBytes, false);
  
  while(Wire.available() < numBytes){  }
  
  for(int i = numBytes-1; i > -1; i--){
    byteMappedCode.data[i] = Wire.read();
  }
  
  /*
  for(int i = 0; i < numBytes; i++){
    byteMappedCode.data[i] = Wire.read();
  }
  return byteMappedCode.code;
  */
}
/*
  Multiplexes to set a different capacitor to funnel to the AD7746
  should probably put a delay after this or to disregard the next conversion.
  NOTE: Should also move this to a separate multiplexer library or look into 
  downloading one to use.
*/
void selectCapacitor(int capIndex){
  
  if(capIndex >= 0 && capIndex < 8){
     switch(capIndex){
        case 0:                            // A0 channel, pads 1 and 9
            digitalWrite(TOP_S0_PIN,LOW);
            digitalWrite(TOP_S1_PIN,LOW);
            digitalWrite(TOP_S2_PIN,LOW);
            
            digitalWrite(BOT_S0_PIN,LOW);
            digitalWrite(BOT_S1_PIN,LOW);
            digitalWrite(BOT_S2_PIN,LOW);
            break;
        case 1:                            // A1 channel, pads 2 and 10
            digitalWrite(TOP_S0_PIN,HIGH);
            digitalWrite(TOP_S1_PIN,LOW);
            digitalWrite(TOP_S2_PIN,LOW);
            
            digitalWrite(BOT_S0_PIN,HIGH);
            digitalWrite(BOT_S1_PIN,LOW);
            digitalWrite(BOT_S2_PIN,LOW);
            break;
        case 2:                            // A2 channel, pads 3 and 11
            digitalWrite(TOP_S0_PIN,LOW);
            digitalWrite(TOP_S1_PIN,HIGH);
            digitalWrite(TOP_S2_PIN,LOW);
            
            digitalWrite(BOT_S0_PIN,LOW);
            digitalWrite(BOT_S1_PIN,HIGH);
            digitalWrite(BOT_S2_PIN,LOW);
            break;
        case 3:                            // A3 channel, pads 4 and 12
            digitalWrite(TOP_S0_PIN,HIGH);
            digitalWrite(TOP_S1_PIN,HIGH);
            digitalWrite(TOP_S2_PIN,LOW);
            
            digitalWrite(BOT_S0_PIN,HIGH);
            digitalWrite(BOT_S1_PIN,HIGH);
            digitalWrite(BOT_S2_PIN,LOW);
            break;
        case 4:                            // A4 channel, pads 5 and 13
            digitalWrite(TOP_S0_PIN,LOW);
            digitalWrite(TOP_S1_PIN,LOW);
            digitalWrite(TOP_S2_PIN,HIGH);
            
            digitalWrite(BOT_S0_PIN,LOW);
            digitalWrite(BOT_S1_PIN,LOW);
            digitalWrite(BOT_S2_PIN,HIGH);
            break;
        case 5:                            // A5 channel, pads 6 and 14
            digitalWrite(TOP_S0_PIN,HIGH);
            digitalWrite(TOP_S1_PIN,LOW);
            digitalWrite(TOP_S2_PIN,HIGH);
            
            digitalWrite(BOT_S0_PIN,HIGH);
            digitalWrite(BOT_S1_PIN,LOW);
            digitalWrite(BOT_S2_PIN,HIGH);
            break;
        case 6:                            // A6 channel, pads 7 and 15
            digitalWrite(TOP_S0_PIN,LOW);
            digitalWrite(TOP_S1_PIN,HIGH);
            digitalWrite(TOP_S2_PIN,HIGH);
            
            digitalWrite(BOT_S0_PIN,LOW);
            digitalWrite(BOT_S1_PIN,HIGH);
            digitalWrite(BOT_S2_PIN,HIGH);
            break;
        case 7:                            // A7 channel, pads 8 and 16
            digitalWrite(TOP_S0_PIN,HIGH);
            digitalWrite(TOP_S1_PIN,HIGH);
            digitalWrite(TOP_S2_PIN,HIGH);
            
            digitalWrite(BOT_S0_PIN,HIGH);
            digitalWrite(BOT_S1_PIN,HIGH);
            digitalWrite(BOT_S2_PIN,HIGH);
            break;
     }
    
  } else{
    Serial.println("capIndex out of range"); 
  }
  
}


////////////////////////////////////////////////////////////////////////////
void calibrate(int index) 
{
  calibration[index] = 0;
  writeConfig(REGISTER_CAP_DAC_A, _BV(7) | calibration[index]);

  Serial.println("Cal CapDAC A");
//  Serial.println("Value");

  long value = readValue();
//  Serial.println(value);

  while (value>VALUE_LOWER_BOUND && calibration[index] < 128) {
    calibration[index]++;
    writeConfig(REGISTER_CAP_DAC_A, _BV(7) | calibration[index]);
    value = readValue();
//    Serial.println(value);
  }
//  Serial.println("Index");
  Serial.println(calibration[index]);
  Serial.println("done");
}
unsigned char readRegister(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
  while(Wire.available()==0) {
    // waiting
  }
  v = Wire.read();
  return v;
}
long readValue() 
{
  long ret = 0;
  uint8_t data[3];

  char status = 0;
  //wait until a conversion is done
  while (!(status & (_BV(0) | _BV(2)))) {
    //wait for the next conversion
    status= readRegister(0x00);
  }

  unsigned long value =  readLong(0x01);

  value >>=8;
  //we have read one byte to much, now we have to get rid of it
  ret =  value;

  return ret;
}
unsigned long readLong(unsigned char r) 
{
  union {
    char data[4];
    unsigned long value;
  } 
  byteMappedLong;

  byteMappedLong.value = 0L;

  Wire.beginTransmission(I2C_ADDRESS); // begin read cycle
  Wire.write(0); //pointer to first data register
  Wire.endTransmission(); // end cycle
  //the data pointer is reset anyway - so read from 0 on

  Wire.requestFrom(I2C_ADDRESS,r+4); // reads 2 bytes plus all bytes before the register

    while (!Wire.available()==r+4) {
      ; //wait
    }
  for (int i=r+3; i>=0; i--) {
    uint8_t c = Wire.read();
    if (i<4) {
      byteMappedLong.data[i]= c;
    }
  }

  return byteMappedLong.value;

}
