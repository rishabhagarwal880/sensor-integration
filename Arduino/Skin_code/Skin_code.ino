#include <AD7746.h>

#include <AD7746.h>

    #include <AD7746.h>
    #include <Wire.h>
    //#include <ros.h>
    //#include <std_msgs/String.h>
    
    //ros::NodeHandle nh;
    
    //std_msgs::String str_msg;
    //ros::Publisher chatter("chatter", &str_msg);
    
    uint32_t c, c2;
    float calibrated;
    
    AD7746 capac;
    void setup() {
      //nh.initNode();
      //nh.advertise(chatter);
      Wire.begin();
      Serial.begin(9600);
      capac.initialize();
      Serial.println("Initializing");
      delay(1000);
      if (capac.testConnection()) {
        capac.writeCapSetupRegister(0x80);
        capac.writeVtSetupRegister(0x00);
        capac.writeExcSetupRegister(0x0B);
        capac.writeConfigurationRegister(0xA2);
        for (int i = 0; i < 50; i++) {
          c = capac.getCapacitance() - 0x800000;
          calibrated = float(c);
          calibrated = calibrated / 10000;
          delay(100);
        }
      }
      Serial.println("Done");
      Serial.print("calibrated value is: ");
      Serial.println(calibrated);
    }
    
    void loop() {
      uint32_t c;
      float d;
      if (capac.testConnection()) {
        capac.writeCapSetupRegister(0x80);
        capac.writeVtSetupRegister(0x00);
        capac.writeExcSetupRegister(0x0B);
        capac.writeConfigurationRegister(0xA2);
        c = capac.getCapacitance() - 0x800000;
        d = float(c);
        d = d / 10000;
        Serial.println(d);
        /*
        if (d > calibrated + 2) {
          str_msg.data = "Positive";
          Serial.println("Positive");
        }
        */
        if (d < calibrated - 0.4) {
          //str_msg.data = "Negative";
          Serial.println("Negative");
        } else {
            //str_msg.data = "Positive";
            Serial.println("Positive");
        }
    
        //chatter.publish( &str_msg );
        //nh.spinOnce();
        delay(100);
      }
    }
