// Program to send z acceleration of mpu6050 over serial as a byte array

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// mpu6050 object for interaction with the sensor
MPU6050 accelgyro;

// define the LED pin and the initial blink state
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// time and acceleration vars
int16_t az;
unsigned long time = 0;     // time value 
unsigned long time_now = 0;     // time value 
byte byte_data_t[4]; // byte array for time
byte byte_data_z[2]; // byte array for acceleration
/*bool kalman = false;*/


// set kalman filter parameters
/*float Q = 100;*/
/*float R = 10;*/
/*float P = 0;*/
/*float x = 0;*/
/*float S, K, v;*/
/*int16_t az_kalman;*/
/*byte byte_data_k[2]; // byte array for filtered acceleration*/


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

    // set output range
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

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

    /*if (kalman == true) {*/
        /*// predict*/
        /*x = x;*/
        /*P = P + Q;*/
        /*// update*/
        /*v = (float) az - x;*/
        /*S = P + R;*/
        /*K = P / S;*/
        /*x = x + K * v;*/
        /*P = P - K * S * K;*/
        /*byte byte_data_k[2];*/
        /*az_kalman = (int16_t) x;*/
        /*byte_data_k[0] = az_kalman & 255;*/
        /*byte_data_k[1] = (az_kalman >> 8) & 255;*/
    /*}*/

    // send data as byte array over serial
    byte_data_t[0] = time & 255;
    byte_data_t[1] = (time >> 8) & 255;
    byte_data_t[2] = (time >> 16) & 255;
    byte_data_t[3] = (time >> 32) & 255;
    byte_data_z[0] = az & 255;
    byte_data_z[1] = (az >> 8) & 255;
    /*if (kalman == true) {*/
        /*byte buf[8] = {byte_data_t[0], byte_data_t[1], byte_data_t[2], byte_data_t[3],*/
                        /*byte_data_z[0], byte_data_z[1],*/
                        /*byte_data_k[0], byte_data_k[1]};*/
        /*Serial.write(buf, 8);*/
    /*} else {*/
    byte buf[6] = {byte_data_t[0], byte_data_t[1], byte_data_t[2], byte_data_t[3],
                    byte_data_z[0], byte_data_z[1]};
    Serial.write(buf, 6);
    /*}*/

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
