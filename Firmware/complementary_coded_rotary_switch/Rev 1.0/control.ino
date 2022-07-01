// Firmware version 1.1. Release date: 09.02.2022 //This version of the firmware features a complementary-coded rotary switch

// In no respect shall DanInvents be accountable for any liabilities, claims, demands, damages or suits resulting from the use of
// the flight controller and/or this software. By using this software, you assume all risks associated with this product and
// its associated features. While the circuitry and software have been tested, they should be considered experimental and handled
// with caution.

// Before uploading this code make sure that you have downloaded the latest ADXL343 (Adafruit) and MS5637 (Sparkfun) libraries.
// You will also need the Circular Buffer library by Roberto Lo Giacco.
// Thanks to Adafruit, Sparkfun and Roberto for the open source libraries and also to Homemade Multibody Dynamics for a guide into how to log data fast.
// Thanks to MartinMcC for showing how to use a rotary encoder with a microcontroller.
// Special thanks to Barun Basnet for the exceptional work on Kalman filters.
// Special thanks to Earle Philhower for providing the support that allows using the Arduino libraries and IDE with the RP2040.

#include <Wire.h>
#include "SparkFun_MS5637_Arduino_Library.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <EEPROM.h>
#include <CircularBuffer.h>
#include "pico/stdlib.h"

CircularBuffer <float,100> FilteredAltitudes;
CircularBuffer <float,100> altitudes;
CircularBuffer <float,100> accelerations;
CircularBuffer <long,100> times;

//Initialization of Kalman Variables
float R = 0.3; //R = measurement noise covariance. Larger R means large measurement uncertainty
float Q = 0.3*1e-4;  //Q = process noise covariance. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more
double Xpe0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0;  //Ppe0 = prior estimation of "error covariance" at t=0
double P1,P0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double K, Xe0, Z; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

//Physical magnitudes
float altold; //Baseline pressure
float temp;
float currentPressure;
float altitudeDelta;
float filteredAltitudeDelta;
float rocketAccel; //z axis offset +0.03g
float startingPressure = 0.0;

//Definition of time and auxiliary integers
int tconfig, n, p = 0, r = 0;
int deltat; //Time step of every loop iteration
long int t1; //Time variables
long int t4, tout = 300000; //Here tout is the timeout variable tout = 300000 equals 5 min of data logging time

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345, &Wire1);

//Config. rotary switch. This configuration is for the real-coded rotary switch
byte switchPins[4] = {15, 13, 14, 16}; //Digital pins assigned to the rotary switch
byte rotValue = B0000; // Variable for printing value over serial debug
byte switchPos; // Variable for storing the current switch possition
byte previousValue; //Variable for storing the previous switch possition

//Boolean variables defining the state of the program
bool initVar = true;
bool deploy = false;
bool automatic = false;
bool timer = false;
bool overtime = false;

//LEDs
int batLED = 2; //Battery indicator LED
int statusLED = 26; //Status LED

//Piezo
int piezo = 12;

MS5637 barometricSensor; //Creates a barometricSensor object
File dataFile; //Creates a dataFile object
Servo servo1; //Creates a servo1 object
Servo servo2; //Creates a servo2 object

void setup() {
 // Serial.begin(9600); //For debugging purposes only
  EEPROM.begin(512); //Emulates EEPROM by allocating 512 kB from the flash memory

  //Declaration of the I2C pins
  Wire1.setSDA(10);
  Wire1.setSCL(11);

  //Declaration of the SPI pins
  SPI.setRX(20);
  SPI.setTX(19);
  SPI.setSCK(18);
  SPI.setCS(17);

  //Declaration of the pins for the battery indicator, and status LED as well as the pin for the buzzer
  pinMode(batLED, OUTPUT); //Low battery LED
  pinMode(statusLED, OUTPUT); //Status LED
  pinMode(piezo, OUTPUT); //Piezo buzzer

  //Piezo buzzer PWM settings
  analogWriteFreq(4000); //Set the piezo frequency to 4kHz
  analogWriteRange(100); //Set the dynamic range of the piezo
   
  for (int i = 0; i < 4; i = i + 1){
     pinMode(switchPins[i], INPUT_PULLUP);
  }

  barometricSensor.begin(Wire1);
  //Set the resolution of the sensor to the highest level of resolution: 0.016 mbar //Change this
  barometricSensor.setResolution(ms5637_resolution_osr_1024);
  
  //Take 16 readings and average them
  startingPressure = 0.0;
  for (int x = 0 ; x < 16 ; x++)
    startingPressure += barometricSensor.getPressure();
  startingPressure /= (float)16;

  accel.begin();
  accel.setRange(ADXL343_RANGE_16_G);
  accel.setDataRate(ADXL343_DATARATE_400_HZ);
  switchStartup();
  SDstartup();  //Initialize the SD card  
  preLaunch(); //Here I store the first second of data into the circular buffers
}

void loop() {
  batteryStatus(); //Check the battery level
  
  if (overtime == false){
    currentPressure = barometricSensor.getPressure();
    temp = barometricSensor.getTemperature();
    sensors_event_t event;
    accel.getEvent(&event);
    rocketAccel = ((event.acceleration.y/9.81)-(event.acceleration.x/9.81))/sqrt(2);
    altitudeDelta = barometricSensor.altitudeChange(currentPressure, startingPressure);
    filteredAltitudeDelta = kalmanFilter(altitudeDelta);
  
    if (initVar == true){ //Maybe this should have its own tab
      accelerations.push(rocketAccel);
      altitudes.push(altitudeDelta);
      FilteredAltitudes.push(filteredAltitudeDelta);
      times.push(millis()-t4); //Circular buffer for time

      if(rocketAccel >= 0){
       initVar = false;
        for (int i = 0; i<=99; i++){ //Saving the buffer should be done only once.
          dataFile.print(times[i]-times[0]); //Here times[0] sets the time zero for the time variable
          dataFile.print(',');
          dataFile.print(altitudes.shift());
          dataFile.print(',');
          dataFile.print(FilteredAltitudes.shift());
          dataFile.print(',');
          dataFile.print(accelerations.shift());
          dataFile.print(',');
          dataFile.print(event.acceleration.z/9.81);
          dataFile.print(',');
          dataFile.println(temp, 1);
       }
      dataFile.flush(); //Store data of the full second before launch.
      }  
    }

    else if (initVar == false){
      t1 = millis() - t4 - times[0];
      recovery();  
      dataFile.print(t1);
      dataFile.print(',');
      dataFile.print(altitudeDelta);
      dataFile.print(',');
      dataFile.print(filteredAltitudeDelta);
      dataFile.print(',');
      dataFile.print(rocketAccel);
      dataFile.print(',');
      dataFile.print(event.acceleration.z/9.81);
      dataFile.print(',');
      dataFile.println(temp, 1);
  
      if (r == 200 && overtime == false){ //Here I set the rate at which I send data to the uSD card
        r = 0;
        dataFile.flush();
       }
       r++;
    
      if (t1 >= tout){
        overtime = true;
        dataFile.flush();
        dataFile.close(); //After timeout flush the data to the microSD card and close the file
      }
    }
  }
 beepnblink();
}
