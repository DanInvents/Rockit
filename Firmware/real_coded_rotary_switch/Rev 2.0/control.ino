// Firmware version 2. Release date: 01.07.2022 //This version of the firmware features a real-coded rotary switch.

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

// Firmware improvements over the previous version:

// Changed the sign of the longitudinal acceleration. Now positive acceleration is pointing downwards and negative upwards.
// Changed the way that launch is detected. Now the altitude must be greater than 10 m and the acceleration higher than 2 gs for over 100 ms.
// Modified the Kalman filter parameters. Now the filtered data closely follows the measured values but featuring lower noise. This guarantees accurate apogee detection.
// Modified the frequency at which the flight computer beeps, now it beeps less frequently before launch.
// Now the flight computer goes silent once launch is detected. After 5 minutes, the flight computer beeps and flashes the altitude.
// For example, 5 beeps/flashes followed by 7 beeps/flashes means 57 meters.
// Now the flight computer can rotate a servo 180 degrees (not yet tested).
// When set to a possition between 

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
float Q = 0.3*1e-2;  //Q = process noise covariance. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more
double Xpe0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0;  //Ppe0 = prior estimation of "error covariance" at t=0
double P1,P0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double K, Xe0, Z; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

//Physical magnitudes
float altold; //Baseline pressure
int altMax; //Rounded maximum altitude
int altMaxDig[4] = {}; //Max altitude digits
int rmnd; //Dummy variable remainder
int dvsr; //Dummy variable for beeping/flasing the altitude
float temp;
float currentPressure;
float altitudeDelta;
float altThreshold = 10.0; //Altitude threshold for launchd detection in meters
float accelThreshold = 2.0; //Acceleration threshold for launch detection in gs.
float filteredAltitudeDelta;
float rocketAccel;
float startingPressure = 0.0;

//Definition of time and auxiliary integers
int tconfig, n, q, p = 0, r = 0;
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
bool launchCondition1 = false;
bool launchCondition2 = false;
bool deploy = false;
bool automatic = false;
bool timer = false;
bool overtime = false;
bool piezoEnable = true;

//LEDs
int batLED = 2; //Battery indicator LED
int statusLED = 26; //Status LED

//Servos
int servo1pin = 28;
int servo2pin = 27;

//Piezo
int piezo = 12;

MS5637 barometricSensor; //Creates a barometricSensor object
File dataFile; //Creates a dataFile object
Servo servo1; //Creates a servo1 object
Servo servo2; //Creates a servo2 object

void setup() {
  //Serial.begin(9600); //For debugging purposes only
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
    rocketAccel = -((event.acceleration.y/9.81)-(event.acceleration.x/9.81))/sqrt(2);
    altitudeDelta = barometricSensor.altitudeChange(currentPressure, startingPressure);
    filteredAltitudeDelta = kalmanFilter(altitudeDelta);
    

    if (altitudeDelta > altThreshold && launchCondition1 == false){ //Threshold condition set to 10 m
      launchCondition1 = true;
    }
    
    if (rocketAccel > accelThreshold && launchCondition2 == false){
      q++;
      if (q > 10){ //launcCondition2 stablishes the requirement that to detect launch there should be at least an acceleration of 2g for 100 ms
        launchCondition2 = true;
      }
    }

    else if (rocketAccel < accelThreshold && launchCondition2 == false){
      q = 0;
    }

    if (initVar == true){ //Store data to the circular buffer
      accelerations.push(rocketAccel);
      altitudes.push(altitudeDelta);
      FilteredAltitudes.push(filteredAltitudeDelta);
      times.push(millis()-t4); //Circular buffer for time

      if (launchCondition1 == true && launchCondition2 == true){
        initVar = false;
          
        for (int i = 0; i<=99; i++){ //Saving the buffer allows me to store the data measured before launch.
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
          
         dataFile.flush(); //Store data of the 908 ms before launch
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

      if (altitudeDelta > altold){ //Here is where I store the maximum altitude value
        altMax = round(altitudeDelta);
        altold = altMax;
      }
  
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
