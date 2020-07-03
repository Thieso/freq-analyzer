// program to send z acceleration of mpu6050 over serial as a byte array

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 accelgyro;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// orientation/motion vars
int16_t az;
unsigned long time = 0;     // time value 
unsigned long time_now = 0;     // time value 
int period = 1;        // period at which to send the signal


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    accelgyro.initialize();

    // set low pass filter
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);

    // configure Arduino LED for blinking
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // read raw data from accelerometer
    az = accelgyro.getAccelerationZ();

    // get time in milliseconds
    time = millis();

    // if time passed the period, send data over serial
    if (time >= time_now + period) {
        // time += period;
        // set time 
        time_now = millis();
        // send data as byte array over serial
        byte byte_data_t[4];
        byte_data_t[0] = time & 255;
        byte_data_t[1] = (time >> 8) & 255;
        byte_data_t[2] = (time >> 16) & 255;
        byte_data_t[3] = (time >> 32) & 255;
        byte byte_data_z[2];
        byte_data_z[0] = az & 255;
        byte_data_z[1] = (az >> 8) & 255;
        byte buf[6] = {byte_data_t[0], byte_data_t[1], byte_data_t[2], byte_data_t[3],
                        byte_data_z[0], byte_data_z[1]};
        Serial.write(buf, 6);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


/*#include<Wire.h>*/

/*const int MPU6050_addr=0x68;*/
/*int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;*/
/*void setup(){*/
  /*Wire.begin();*/
  /*Wire.beginTransmission(MPU6050_addr);*/
  /*Wire.write(0x6B);*/
  /*Wire.write(0);*/
  /*Wire.endTransmission(true);*/
  /*Serial.begin(115200);*/
/*}*/

/*void loop(){*/
    /*Wire.beginTransmission(MPU6050_addr);*/
    /*Wire.write(0x3B);*/
    /*Wire.endTransmission(false);*/
    /*Wire.requestFrom(MPU6050_addr,14,true);*/
    /*AccX=Wire.read()<<8|Wire.read();*/
    /*AccY=Wire.read()<<8|Wire.read();*/
    /*AccZ=Wire.read()<<8|Wire.read();*/
    /*Temp=Wire.read()<<8|Wire.read();*/
    /*GyroX=Wire.read()<<8|Wire.read();*/
    /*GyroY=Wire.read()<<8|Wire.read();*/
    /*GyroZ=Wire.read()<<8|Wire.read();*/
    /*[>Serial.print("AccX = "); Serial.print(AccX);<]*/
    /*[>Serial.print(" || AccY = "); Serial.print(AccY);<]*/
    /*[>Serial.print(" || AccZ = "); Serial.print(AccZ);<]*/
    /*[>Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);<]*/
    /*[>Serial.print(" || GyroX = "); Serial.print(GyroX);<]*/
    /*[>Serial.print(" || GyroY = "); Serial.print(GyroY);<]*/
    /*[>Serial.print(" || GyroZ = "); Serial.println(GyroZ);<]*/
    /*time = millis();*/
    /*byte byte_data_t[4];*/
    /*byte_data_t[0] = time & 255;*/
    /*byte_data_t[1] = (time >> 8) & 255;*/
    /*byte_data_t[2] = (time >> 16) & 255;*/
    /*byte_data_t[3] = (time >> 32) & 255;*/
    /*byte byte_data_z[2];*/
    /*byte_data_z[0] = AccZ & 255;*/
    /*byte_data_z[1] = (AccZ >> 8) & 255;*/
    /*byte buf[6] = {byte_data_t[0], byte_data_t[1], byte_data_t[2], byte_data_t[3],*/
                    /*byte_data_z[0], byte_data_z[1]};*/
    /*Serial.write(buf, 6);*/
/*}*/
