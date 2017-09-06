/******************************************************************************
 i2ctest.cpp
 Raspberry Pi I2C interface demo
 Byron Jacquot @ SparkFun Electronics>
 4/2/2014
 https://github.com/sparkfun/Pi_Wedge
 
 A brief demonstration of the Raspberry Pi I2C interface, using the SparkFun
 Pi Wedge breakout board.
 
 Resources:
 
 This example makes use of the Wiring Pi library, which streamlines the interface
 the the I/O pins on the Raspberry Pi, providing an API that is similar to the
 Arduino.  You can learn about installing Wiring Pi here:
 http://wiringpi.com/download-and-install/
 
 The I2C API is documented here:
 https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/
 
 The init call returns a standard file descriptor.  More detailed configuration
 of the interface can be performed using ioctl calls on that descriptor.
 See the wiringPi I2C implementation (wiringPi/wiringPiI2C.c) for some examples.
 Parameters configurable with ioctl are documented here:
 http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c/dev-interface
 
 Hardware connections:
 
 This file interfaces with the SparkFun MCP4725 breakout board:
 https://www.sparkfun.com/products/8736
 
 The board was connected as follows:
 (Raspberry Pi)(MCP4725)
 GND  -> GND
 3.3V -> Vcc
 SCL  -> SCL
 SDA  -> SDA
 
 An oscilloscope probe was connected to the analog output pin of the MCP4725.
 
 To build this file, I use the command:
 >  g++ i2ctest.cpp -lwiringPi
 
 Then to run it, first the I2C kernel module needs to be loaded.  This can be
 done using the GPIO utility.
 > gpio load i2c 400
 > ./a.out
 
 This will run the MCP through its output range several times.  A rising
 sawtooth will be seen on the analog output.
 
 Development environment specifics:
 Tested on Raspberry Pi V2 hardware, running Raspbian.
 Building with GCC 4.6.3 (Debian 4.6.3-14+rpi1)
 
 This code is beerware; if you see me (or any other SparkFun employee) at the
 local, and you've found our code helpful, please buy us a round!
 
 Distributed as-is; no warranty is given.
 ******************************************************************************/

#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
//#include <AD7746.h>

using namespace std;

int main()
{
    int fd, result;
    //AD7746 device = AD7746();
    //device.initialize();
    
    // Initialize the interface by giving it an external device ID.
    // The MCP4725 defaults to address 0x60.
    //
    // It returns a standard file descriptor.
    //
    fd = wiringPiI2CSetup(0x60);
    
    cout << "Init result: "<< fd << endl;
    
    for(int i = 0; i < 0x0000ffff; i++)
    {
        // I tried using the "fast write" command, but couldn't get it to work.
        // It's not entirely obvious what's happening behind the scenes as
        // regards to endianness or length of data sent.  I think it's only
        // sending one byte, when we really need two.
        //
        // So instead I'm doing a 16 bit register access.  It appears to
        // properly handle the endianness, and the length is specified by the
        // call.  The only question was the register address, which is the
        // concatenation of the command (010x = write DAC output)
        // and power down (x00x = power up) bits.
        wiringPiI2CWriteReg16(fd, 0x07, 0x80);
        wiringPiI2CWriteReg16(fd, 0x08, 0x00);
        wiringPiI2CWriteReg16(fd, 0x09, 0x0B);
        wiringPiI2CWriteReg16(fd, 0x0A, 0xA2);
        result = wiringPiI2CReadReg16(fd, 0);
        // result = wiringPiI2CWriteReg16(fd, 0x40, (i & 0xfff) );
        
        //if(result == -1)
        //{
          //  cout << "Error.  Errno is: " << errno << endl;
        //}
    }
}
