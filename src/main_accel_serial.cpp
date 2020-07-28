// Program to send z acceleration of mpu6050 over serial as a byte array

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// define logging output
#define LOG
// define logging file name
#define FILE_BASE_NAME "Data"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

// mpu6050 object for interaction with the sensor
MPU6050 accelgyro;

// chip select pin
const int chip_select = 10;

// define the LED pin and the initial blink state
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// time and acceleration vars
int16_t az;
unsigned long time = 0;     // time value 
unsigned long time_now = 0;     // time value 
byte byte_data_t[4]; // byte array for time
byte byte_data_z[2]; // byte array for acceleration
String data_string;
File data_file;
char fileName[13] = FILE_BASE_NAME "00.csv";


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

    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    // wait for Serial Monitor to connect. Needed for native USB port boards only:
    while (!Serial);

    #ifdef LOG
        Serial.print("Initializing SD card...");
        if (!SD.begin(chip_select))
        {
            Serial.println("initialization failed. Things to check:");
            Serial.println("1. is a card inserted?");
            Serial.println("2. is your wiring correct?");
            Serial.println("3. did you change the chipSelect pin to match your shield or module?");
            Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
            while (true);
        }
        Serial.println("initialization done.");
        // find file name for the log file (increase the number if necessary)
        while (SD.exists(fileName))
        {
            if (fileName[BASE_NAME_SIZE + 1] != '9')
            {
                fileName[BASE_NAME_SIZE + 1]++;
            }
            else if (fileName[BASE_NAME_SIZE] != '9')
            {
                fileName[BASE_NAME_SIZE + 1] = '0';
                fileName[BASE_NAME_SIZE]++;
            }
            else
            {
                Serial.println("Can't create file name");
            }
        }
#endif

    // initialize device
    accelgyro.initialize();

    // set low pass filter
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);

    // set output range
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

    // open file for writing
    data_file = SD.open(fileName, FILE_WRITE);

    // configure Arduino LED for blinking
    pinMode(LED_PIN, OUTPUT);

    // display cancel message
    Serial.print("Logfile: ");
    Serial.println(fileName);
    Serial.println("Starting to log, cancel with keypress");
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // read raw data from accelerometer
    az = accelgyro.getAccelerationZ();

    // get time in microseconds
    time = micros();

    #ifdef LOG
        data_string = String(time) + "," + String(az);

        // open file
        // if the file is available, write to it:
        if (data_file)
        {
            data_file.println(data_string);
        }
        // if the file isn't open, pop up an error:
        else
        {
            Serial.print("error opening ");
            Serial.println(fileName);
        }

        // close file if key pressed
        if(Serial.available()) {
            Serial.println("Done");
            data_file.close();
            while(true){
                delay(100);
            }
        }

    #else
        // send data as byte array over serial
        byte_data_t[0] = time & 255;
        byte_data_t[1] = (time >> 8) & 255;
        byte_data_t[2] = (time >> 16) & 255;
        byte_data_t[3] = (time >> 32) & 255;
        byte_data_z[0] = az & 255;
        byte_data_z[1] = (az >> 8) & 255;
        byte buf[6] = {byte_data_t[0], byte_data_t[1], byte_data_t[2], byte_data_t[3],
                        byte_data_z[0], byte_data_z[1]};
        Serial.write(buf, 6);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
