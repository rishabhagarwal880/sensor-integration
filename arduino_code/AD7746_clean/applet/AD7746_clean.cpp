#include <Wire.h>

//AD7746 definitions
#define I2C_ADDRESS  0x48 //0x90 shift one to the rigth

#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_VT_DATA 0x04
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_VT_SETUP 0x08
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A
#define REGISTER_CAP_DAC_A 0x0B
#define REGISTER_CAP_DAC_B 0x0B
#define REGISTER_CAP_OFFSET 0x0D
#define REGISTER_CAP_GAIN 0x0F
#define REGISTER_VOLTAGE_GAIN 0x11

#define RESET_ADDRESS 0xBF

#define VALUE_UPPER_BOUND 16000000L 
#define VALUE_LOWER_BOUND 0xFL
#define MAX_OUT_OF_RANGE_COUNT 3
#define CALIBRATION_INCREASE 1
#include "WProgram.h"
void setup();
void loop();
void calibrate (byte direction);
void calibrate();
long readValue();
void writeRegister(unsigned char r, unsigned char v);
void writeInteger(unsigned char r, unsigned int v);
unsigned char readRegister(unsigned char r);
void readRegisters(unsigned char r, unsigned int numberOfBytes, unsigned char buffer[]);
unsigned int readInteger(unsigned char r);
unsigned long readLong(unsigned char r);
void displayStatus();
byte calibration;
byte outOfRangeCount = 0;

unsigned long offset = 0;

void setup()
{

  Wire.begin(); // sets up i2c for operation
  Serial.begin(9600); // set up baud rate for serial

  Serial.println("Initializing");

  Wire.beginTransmission(I2C_ADDRESS); // start i2c cycle
  Wire.send(RESET_ADDRESS); // reset the device
  Wire.endTransmission(); // ends i2c cycle

  //wait a tad for reboot
  delay(1);

  writeRegister(REGISTER_EXC_SETUP, _BV(3) | _BV(1) | _BV(0)); // EXC source A

  writeRegister(REGISTER_CAP_SETUP,_BV(7)); // cap setup reg - cap enabled

  Serial.println("Getting offset");
  offset = ((unsigned long)readInteger(REGISTER_CAP_OFFSET)) << 8;  
  Serial.print("Factory offset: ");
  Serial.println(offset);

  writeRegister(0x0A, _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2) | _BV(0));  // set configuration to calib. mode, slow sample

  //wait for calibration
  delay(10);

  displayStatus();
  Serial.print("Calibrated offset: ");
  offset = ((unsigned long)readInteger(REGISTER_CAP_OFFSET)) << 8;  
  Serial.println(offset);

  writeRegister(REGISTER_CAP_SETUP,_BV(7)); // cap setup reg - cap enabled

  writeRegister(REGISTER_EXC_SETUP, _BV(3)); // EXC source A

  writeRegister(REGISTER_CONFIGURATION, _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(0)); // continuous mode

  displayStatus();
  calibrate();


  Serial.println("done");
}

void loop() // main program begins
{

  // if we recieve date we print out the status
  if (Serial.available() > 0) {
    // read the incoming byte:
    Serial.read();
    displayStatus();

  }

  long value = readValue();
  Serial.print(offset);
  Serial.print("/");
  Serial.print((int)calibration);
  Serial.print("/");
  Serial.println(value);
  if ((value<VALUE_LOWER_BOUND) or (value>VALUE_UPPER_BOUND)) {
    outOfRangeCount++;
  }
  if (outOfRangeCount>MAX_OUT_OF_RANGE_COUNT) {
    if (value < VALUE_LOWER_BOUND) {
      calibrate(-CALIBRATION_INCREASE);
    } 
    else {
      calibrate(CALIBRATION_INCREASE);
    }
    outOfRangeCount=0;
  }

  delay(50);
}

void calibrate (byte direction) {
  calibration += direction;
  //assure that calibration is in 7 bit range
  calibration &=0x7f;
  writeRegister(REGISTER_CAP_DAC_A, _BV(7) | calibration);
}

void calibrate() {
  calibration = 0;

  Serial.println("Calibrating CapDAC A");

  long value = readValue();

  while (value>VALUE_UPPER_BOUND && calibration < 128) {
    calibration++;
    writeRegister(REGISTER_CAP_DAC_A, _BV(7) | calibration);
    value = readValue();
  }
  Serial.println("done");
}

long readValue() {
  long ret = 0;
  uint8_t data[3];

  char status = 0;
  //wait until a conversion is done
  while (!(status & (_BV(0) | _BV(2)))) {
    //wait for the next conversion
    status= readRegister(REGISTER_STATUS);
  }

  unsigned long value =  readLong(REGISTER_CAP_DATA);

  value >>=8;
  //we have read one byte to much, now we have to get rid of it
  ret =  value;

  return ret;
}


void writeRegister(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);
  Wire.send(v);
  Wire.endTransmission();
}

void writeInteger(unsigned char r, unsigned int v) {
  writeRegister(r,(unsigned byte)v);
  writeRegister(r+1,(unsigned byte)(v>>8));
}

unsigned char readRegister(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
  while(Wire.available()==0) {
    // waiting
  }
  v = Wire.receive();
  return v;
}

void readRegisters(unsigned char r, unsigned int numberOfBytes, unsigned char buffer[])
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.send(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, numberOfBytes); // read a byte
  char i = 0;
  while (i<numberOfBytes) {
    while(!Wire.available()) {
      // waiting
    }
    buffer[i] = Wire.receive();
    i++;
  }
}

unsigned int readInteger(unsigned char r) {
  union {
    char data[2];
    unsigned int value;
  } 
  byteMappedInt;

  byteMappedInt.value = 0;

  Wire.beginTransmission(I2C_ADDRESS); // begin read cycle
  Wire.send(0); //pointer to first cap data register
  Wire.endTransmission(); // end cycle
  //after this, the address pointer is set to 0 - since a stop condition has been sent

  Wire.requestFrom(I2C_ADDRESS,r+2); // reads 2 bytes plus all bytes before the register

    while (!Wire.available()==r+2) {
      ; //wait
    }

  for (int i=r+1; i>=0; i--) {
    uint8_t c = Wire.receive();
    if (i<2) {
      byteMappedInt.data[i]= c;
    }
  }

  return byteMappedInt.value;

}

unsigned long readLong(unsigned char r) {
  union {
    char data[4];
    unsigned long value;
  } 
  byteMappedLong;

  byteMappedLong.value = 0L;

  Wire.beginTransmission(I2C_ADDRESS); // begin read cycle
  Wire.send(0); //pointer to first data register
  Wire.endTransmission(); // end cycle
  //the data pointer is reset anyway - so read from 0 on

  Wire.requestFrom(I2C_ADDRESS,r+4); // reads 2 bytes plus all bytes before the register

    while (!Wire.available()==r+4) {
      ; //wait
    }
  for (int i=r+3; i>=0; i--) {
    uint8_t c = Wire.receive();
    if (i<4) {
      byteMappedLong.data[i]= c;
    }
  }

  return byteMappedLong.value;

}

void displayStatus() {
  unsigned char data[18];
  
  readRegisters(0,18,data);
  
  Serial.println("\nAD7746 Registers:");
  Serial.print("Status (0x0): ");
  Serial.println(data[0],BIN);
  Serial.print("Cap Data (0x1-0x3): ");
  Serial.print(data[1],BIN);
  Serial.print(".");
  Serial.print(data[2],BIN);
  Serial.print(".");
  Serial.println(data[3],BIN);
  Serial.print("VT Data (0x4-0x6): ");
  Serial.print(data[4],BIN);
  Serial.print(".");
  Serial.print(data[5],BIN);
  Serial.print(".");
  Serial.println(data[6],BIN);
  Serial.print("Cap Setup (0x7): ");
  Serial.println(data[7],BIN);
    Serial.print("VT Setup (0x8): ");
  Serial.println(data[8],BIN);
  Serial.print("EXC Setup (0x9): ");
  Serial.println(data[9],BIN);
  Serial.print("Configuration (0xa): ");
  Serial.println(data[10],BIN);
  Serial.print("Cap Dac A (0xb): ");
  Serial.println(data[11],BIN);
  Serial.print("Cap Dac B (0xc): ");
  Serial.println(data[12],BIN);
  Serial.print("Cap Offset (0xd-0xe): ");
  Serial.print(data[13],BIN);
  Serial.print(".");
  Serial.println(data[14],BIN);
  Serial.print("Cap Gain (0xf-0x10): ");
  Serial.print(data[15],BIN);
  Serial.print(".");
  Serial.println(data[16],BIN);
  Serial.print("Volt Gain (0x11-0x12): ");
  Serial.print(data[17],BIN);
  Serial.print(".");
  Serial.println(data[18],BIN);
  
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

