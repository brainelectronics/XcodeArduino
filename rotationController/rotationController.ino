// 
// BLDC-Controller-Software
//
// Control BLDC motor via the MC33035 Chip
// Interface via RS485, I2C, Serial and RX/TX
// external 8 bit I/O ports, 64kB EEPROM, 12 bit ADC, 30A current sensor and MPU6050 included

// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Jonas Scharpf
// 				brainelectronics
//
// Date			14.01.16 13:45
// Version		<#version#>
#define VERSION 1.0             // the version of the code
// 
// Copyright	© Jonas Scharpf, 2016
// License		<#license#>
//
// See			ReadMe.txt for further references
//

// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"   
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif

// Include application, user and local libraries
#include "LocalLibrary.h"
#include <Wire.h>
//#include "DHT.h"
//#include "Stepper.h"
//#include "LiquidCrystal.h"
//#include "RTClib.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
// Prototypes


// Define pins
//
// digital pins
#define photocellRotation   2   // output of the rotation photocell connected to INT0/D2
#define photocellStop       3   // output of the stop photocell connected to INT1/D3
#define photocell3          4   // output of the third free photocell connected to D4
#define pwmPin              5   // free PWM output of the Arduino connected to D5
#define s0mux               6   // S0 input pin of the MUX connected to D6
#define s1mux               7   // S1 input pin of the MUX connected to D7
#define s2mux               8   // S2 input pin of the MUX connected to D8
#define s3mux               9   // S3 input pin of the MUX connected to D9
#define latchPin            9   // latch pin of the registers connected to D9
#define lowPassPWM          10  // low pass filtered PWM output of the Arduino connected to D10
#define directionSetPin     11  // select fwd/rev rotation of the BLDC, connected to D11
#define breakPin            12  // pull HIGH to break the motor, connected to D12
#define clock595Pin         13  // clock pin of the shift register connected to D13
#define statusLED           13  // status LED connected to D13
// analog pins
uint8_t generalAnalogInput;     // analog input connected to the MUX output (A0)
uint8_t dataOutputPin;          // shift out the register data (A1)
uint8_t errorMotorPin;          // input of an occured motor error (A2)
uint8_t referencePin;           // read current reference voltage at A3
/*
uint8_t currentMeasurePin;
uint8_t relaisK1;   // relais 1 to turn ESC on/off
uint8_t relaisK2;   // relais 2 to turn ESC on/off
uint8_t photocellRotation;  // photocell to detect rotation of brushless
uint8_t photocellStop;  // photocell to detect stop point at max. angel
uint8_t statusLED;  // blink LED pin 13
#define DHTPIN 7 // DHT11 pin
*/

// Define variables and constants
//
float offSetCurrentSensor = 0;          // offset of ACS712 measured at every (re)start
uint8_t debugMode = 0;                  // default false, set to true to show additional debug info

uint8_t eepromI2cAdress = 0;            // adress at EEPROM to save/read I2C adress of this Arduino
uint8_t eepromDebugMode = 1;            // adress at EEPROM to save/read debug mode
uint8_t eepromStartup = 2;              // adress at EEPROM to save/read number of startups
uint8_t eepromShutdown = 3;             // adress at EEPROM to save/read number of shutdowns
uint8_t eepromCurrentPosition = 4;      // adress at EEPROM to save/read position at shutdown
uint8_t i2cAdress = 0;                  // basic startup I2C Adress
uint8_t numberOfShutdowns = 0;             // number of proper shutdowns
uint8_t numberOfStartups = 0;           // number of startups
/*
int16_t measuredAmp = 0;  // result is in mA
float resolutionArduino = 5000/1023;   // mV
float resolutionChip = 0.066;  // V/A
float offSet = 0;   // offset of ACS712 measured at every (re)start
int16_t lastArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array of all 10 measured values
uint8_t loopCounter = 0;   // counter to go through the value array
int32_t averageResult = 0;  // final result of 10 measurements in mA
uint8_t numberOfRotations = 0;  // number of current rotation
uint8_t maximumRotations = 200; // number of maximum rotaions from stop point onwards
uint8_t stepsPerRevolution = 64;    //number of steps per revolution
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
DHT dht(DHTPIN, DHT11);
*/

//
// Brief    output serial info
//
// output given text/variable via serial connection
// define message as debug info and new line behavior
void serialOutput(char* theText, float theValue, byte newLine, byte debugInfo)
{
    if (debugMode)  // show if debug mode is active
    {
        if (debugInfo)  // it is a debug info
        {
            Serial.print(theText);
            if (newLine)
            {
                Serial.println(theValue);
            }
            else
            {
                Serial.print(theValue);
            }
        }
    }
    else    // it is not a debug info (it is a normal info), show it always
    {
        Serial.print(theText);
        if (newLine)
        {
            Serial.println(theValue);
        }
        else
        {
            Serial.print(theValue);
        }
    }
}

//
// Brief	interruption of rotation photocell
//
// Add rotation photocell code
void rotationPhotocellInterrupted()
{
    // Serial.println("Rotation-Photocell interrupted");
    serialOutput("Rotation-Photocell interrupted", 0, 1, 1);
    // count rotations
}

//
// Brief	interruption of stop photocell
//
// Add stop photocell code
void stopPhotocellInterrupted()
{
    // Serial.println("Stop-Photocell interrupted");
    serialOutput("Stop-Photocell interrupted", 0, 1, 1);
    
    // do not rotate any further !!!
    // stop ESC and set number of rotations to zero
}

//
// Brief    estimate offset of current sensor
//
// estimate the offset of the current sensor ACS712 at every
// startup of the controller over several iterations to get
// a correct current measurement
void estimateCurrentOffset()
{
    /*
    for (uint8_t i=0; i<100; i++)
    {
        offSetCurrentSensor = offSetCurrentSensor + (analogRead(currentMeasurePin) * resolutionArduino)/100;
        delay(10);
    }
    // Serial.print("Estimate offset (mV): " );
    // Serial.println(offSetCurrentSensor);
    */
    serialOutput("Estimated offset at ACS712 (mV): ", offSetCurrentSensor, 1, 1);
}
/*
//
// Brief	measure current with ACS712
//
// Add measure current code
void measureCurrent()
{
    measuredAmp = ((int)(analogRead(currentMeasurePin) * resolutionArduino - offSet)/resolutionChip);
    Serial.print("measured now (mA): ");
    Serial.println(measuredAmp);
    
    measuredAmp = 0;
    for (byte b = 0; b < 10; b++)
    {
        measuredAmp += (int)((analogRead(currentMeasurePin) * resolutionArduino - offSet)/resolutionChip);
        delay(10);
    }
    measuredAmp = measuredAmp/10;
    //Serial.print("measured after 10 (mA): ");
    //Serial.println(measuredAmp);
    
    lastArray[loopCounter] = measuredAmp;
    
    //Serial.println("Array: ");
    for (byte c = 0; c < 10; c++)
    {
        averageResult = averageResult + lastArray[c];
        //Serial.println(lastArray[c]);
    }
    averageResult = averageResult/10;
    
    Serial.print("Measured avg. after 1 sec (mA): ");
    Serial.println(averageResult);
    //Serial.print("loop #");
    //Serial.println(loopCounter);
    
    if (loopCounter < 9)
    {
        loopCounter++;
    }
    else
    {
        loopCounter = 0;    // reset loop counter
    }
    delay(1000);
}
*/
/*
//
// Brief    measure humidity and temperature with DHT11
//
// Add measure dht code
void measureDHT()
{
    // Wait a few seconds between measurements.
    delay(2000);
    
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit
    float f = dht.readTemperature(true);
    
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    
    // Compute heat index
    // Must send in temp in Fahrenheit!
    // convert back to celsius
    // (°F − 32) / 1,8
    float hi = (dht.computeHeatIndex(f, h)-32)/1.8;
    
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print("*C %\t");
    //Serial.print(f);
    //Serial.print("*F\t");
    Serial.print("Feels like: ");
    Serial.print(hi);
    Serial.println("*C");
}
*/

//
// Brief	receive something via I2C
//
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveI2CEvent(int howMany)
{
    while(1 < Wire.available()) // loop through all but the last
    {
        char c = Wire.read(); // receive byte as a character
        Serial.print(c);         // print the character
    }
    int x = Wire.read();    // receive byte as an integer
    Serial.println(x);         // print the integer
}

//
// Brief    read from EEPROM
//
// use this function to read from external I2C EEPROM at given adress
int readEEPROM(uint8_t theAdress)
{
    return EEPROM.read(theAdress);
}

//
// Brief    write to EEPROM
//
// use this function to write to the external I2C EEPROM
// at given adress and with provided value (0 to 255)
void writeEEPROM(uint8_t theAdress, uint8_t theValue)
{
    EEPROM.write(theAdress, theValue);
}

//
// Brief    load parameter from EEPROM
//
// use this function on every startup of the controller
// to load stored setup parameters
void loadEEPROMparameter()
{
    debugMode = readEEPROM(eepromDebugMode);        // read stored debug mode
    i2cAdress = readEEPROM(eepromI2cAdress);        // read stored I2C Adress
    numberOfStartups = readEEPROM(eepromStartup);   // read number of startup
    numberOfShutdowns = readEEPROM(eepromShutdown); // read number of proper shutdowns
    
    // check range of returned value
    if (0 <= debugMode < 2) // error on loading debug mode
    {
        debugMode = 1;
        // char myInfo[50] = "EEPROM loading error on debug mode! Now set to ";
        // serialOutput(myInfo[50], debugMode, 1, 0);
        serialOutput("EEPROM loading error on debug mode! Now set to ", debugMode, 1, 0);
    }
}

// Brief activate shutdown, stop motor and save position
//
// use this function to shut down the controller properly by stopping
// the motor and saving the current position of the motor/mechanical system
void shutdownSystem()
{
    // stop motor
    digitalWrite(breakPin, HIGH);
    delay(500);
    
    // read position, save as HEX!!!
    uint8_t theCurrentPosition = 111;
    
    // save current position
    writeEEPROM(eepromCurrentPosition, theCurrentPosition);
    
    // read number of successful startups and proper shutdowns
    numberOfStartups = readEEPROM(eepromStartup);
    
    // reset number of startups to 0 if it is larger than 250
    if (numberOfStartups > 250)
    {
        numberOfStartups = 0;
        writeEEPROM(eepromStartup, numberOfStartups);   // save the reset back to EEPROM
    }
    
    // save "okay" for proper shutdown
    writeEEPROM(eepromShutdown, numberOfShutdowns);
    
    // tell user "done"/unplug controller-status
    blink(statusLED, 5, 500);
}

//
// Brief    Setup
//
// basic setup code, only executed one time on startup
void setup()
{
    Serial.begin(115200);   // start serial communication
    delay(50);              // wait a little bit to start communication correct
    
    // load saved parameter from EEPROM
    loadEEPROMparameter();
    
    // "hello" with basic info
    Serial.print("This is BLDC Controller V");
    Serial.print(VERSION);                      // print current software verison
    Serial.print(" with debug mode ");
    Serial.print(debugMode);                    // read saved debug mode from EEPROM
    Serial.print(" on I2C bus adress #");
    Serial.print(i2cAdress);                    // read saved I2C adress from EEPROM
    Serial.println();
    
    // startup successfully done, save this startup
    numberOfStartups++;
    writeEEPROM(eepromStartup, numberOfStartups);
    
    // join I2C bus
    Wire.begin(i2cAdress);                      // with address #4
    Wire.onReceive(receiveI2CEvent);            // register event via I2C
    
    // define analog pins
    generalAnalogInput = A0;
    dataOutputPin = A1;
    errorMotorPin = A2;
    referencePin = A3;
    
    // define digital pinModes
    pinMode(photocellRotation, INPUT_PULLUP);   // D2/INT0
    pinMode(photocellStop, INPUT_PULLUP);       // D3/INT1
    pinMode(photocell3, INPUT_PULLUP);          // D4
    pinMode(pwmPin, OUTPUT);                    // D5
    pinMode(s0mux, OUTPUT);                     // D6
    pinMode(s1mux, OUTPUT);                     // D7
    pinMode(s2mux, OUTPUT);                     // D8
    pinMode(s3mux, OUTPUT);                     // D9
    pinMode(latchPin, OUTPUT);                  // D9
    pinMode(lowPassPWM, OUTPUT);                // D10
    pinMode(directionSetPin, OUTPUT);           // D11
    pinMode(breakPin, OUTPUT);                  // D12
    pinMode(clock595Pin, OUTPUT);               // D13
    pinMode(statusLED, OUTPUT);                 // D13
    
    // define analog pinModes
    pinMode(generalAnalogInput, INPUT);         // A0
    pinMode(dataOutputPin, OUTPUT);             // A1
    pinMode(errorMotorPin, INPUT);              // A2
    pinMode(referencePin, INPUT);               // A3
    
    // define interrupt functions
    attachInterrupt(photocellStop, stopPhotocellInterrupted, RISING);
    attachInterrupt(photocellRotation, rotationPhotocellInterrupted, RISING);
    
    // estimate offset of ACS712 current sensor
    estimateCurrentOffset();
    
    // there have been more successfull startups than proper shutdowns
    // to be save, init position of motor/mechanical system
    if ((numberOfStartups-1) != numberOfShutdowns)
    {
        // do init
        // warn user to shut down the controller proper !!!
    }
    
    // tell user ready status
    blink(statusLED, 5, 500);
}

//
// Brief	Loop
//
// this code is executed in a loop throughout runtime
void loop()
{
    /*
    measureCurrent();
    runMotor();
    measureDHT();
     */
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
}

/*
 // master reader
 void setup()
 {
 Wire.begin();        // join i2c bus (address optional for master)
 Serial.begin(9600);  // start serial for output
 }
 
 void loop()
 {
 Wire.requestFrom(2, 6);    // request 6 bytes from slave device #2
 
 while(Wire.available())    // slave may send less than requested
 {
 char c = Wire.read(); // receive a byte as character
 Serial.print(c);         // print the character
 }
 
 delay(500);
 }
*/
/*
 // slave sender
 void setup()
 {
 Wire.begin(2);                // join i2c bus with address #2
 Wire.onRequest(requestEvent); // register event
 }
 
 void loop()
 {
 delay(100);
 }
 
 // function that executes whenever data is requested by master
 // this function is registered as an event, see setup()
 void requestEvent()
 {
 Wire.write("hello "); // respond with message of 6 bytes
 // as expected by master
 }
 */

/*
 // master writer
 void setup()
 {
 Wire.begin(); // join i2c bus (address optional for master)
 }
 
 byte x = 0;
 
 void loop()
 {
 Wire.beginTransmission(4); // transmit to device #4
 Wire.write("x is ");        // sends five bytes
 Wire.write(x);              // sends one byte
 Wire.endTransmission();    // stop transmitting
 
 x++;
 delay(500);
 }
*/
/*
 // slave receiver
 void setup()
 {
 Wire.begin(4);                // join i2c bus with address #4
 Wire.onReceive(receiveEvent); // register event
 Serial.begin(9600);           // start serial for output
 }
 
 void loop()
 {
 delay(100);
 }
 
 // function that executes whenever data is received from master
 // this function is registered as an event, see setup()
 void receiveEvent(int howMany)
 {
 while(1 < Wire.available()) // loop through all but the last
 {
 char c = Wire.read(); // receive byte as a character
 Serial.print(c);         // print the character
 }
 int x = Wire.read();    // receive byte as an integer
 Serial.println(x);         // print the integer
 }
 */