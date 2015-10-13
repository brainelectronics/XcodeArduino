// 
// rotationController 
//
// read inputs and control leg
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Jonas Scharpf
// 				brainelectronics
//
// Date			07.10.15 18:47
// Version		<#version#>
// 
// Copyright	© Jonas Scharpf, 2015
// License		<#license#>
//
// See			ReadMe.txt for references
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
#include "DHT.h"
#include "Stepper.h"
//#include "LiquidCrystal.h"
//#include "RTClib.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Prototypes


// Define pins
//
uint8_t currentMeasurePin;
uint8_t relaisK1;   // relais 1 to turn ESC on/off
uint8_t relaisK2;   // relais 2 to turn ESC on/off
uint8_t photocellRotation;  // photocell to detect rotation of brushless
uint8_t photocellStop;  // photocell to detect stop point at max. angel
uint8_t statusLED;  // blink LED pin 13
#define DHTPIN 7 // DHT11 pin

// Define variables and constants
//
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

//
// Brief	Rotation Photocell interrupted
//
// Add rotation photocell code
void rotationPhotocellInterrupted()
{
    Serial.println("Rotation-Photocell interrupted");
    // count rotations
}

//
// Brief	Stop Photocell interrupted
//
// Add stop photocell code
void stopPhotocellInterrupted()
{
    Serial.println("Stop-Photocell interrupted");
    // do not rotate any further !!!
    // stop ESC and set number of rotations to zero
}

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

//
// Brief	run a stepper motor
//
// Add run motor code
void runMotor()
{
    // step one revolution  in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    delay(500);
    
    // step one revolution in the other direction:
    Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(500);
}

//
// Brief	measure humidity and temperature with DHT11
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

//
// Brief	Setup
//
// Add setup code 
void setup()
{
    Serial.begin(9600);
    Serial.println("This is rotationController");
    
    currentMeasurePin = A0;
    photocellStop = 2;
    photocellRotation = 3;
    relaisK1 = 5;
    relaisK2 = 6;
    statusLED = 13;
    
    pinMode(photocellStop, INPUT_PULLUP);
    pinMode(photocellRotation, INPUT_PULLUP);
    pinMode(relaisK1, OUTPUT);
    pinMode(relaisK2, OUTPUT);
    pinMode(statusLED, OUTPUT);
    
    //attachInterrupt(digitalPinToInterrupt(photocellStop), stopPhotocellInterrupted, RISING);
    //attachInterrupt(digitalPinToInterrupt(photocellRotation), rotationPhotocellInterrupted, RISING);
    attachInterrupt(photocellStop, stopPhotocellInterrupted, RISING);
    attachInterrupt(photocellRotation, rotationPhotocellInterrupted, RISING);
    
    myStepper.setSpeed(60); // set the speed at 60 rpm
    dht.begin();
    
    digitalWrite(relaisK1, LOW);
    digitalWrite(relaisK2, LOW);
    
    digitalWrite(relaisK1, HIGH);
    digitalWrite(relaisK2, HIGH);
    
    blink(statusLED, 5, 500);   // tell user working status
    
    for (uint8_t i=0; i<250; i++)
    {
        offSet = offSet + (analogRead(currentMeasurePin) * resolutionArduino)/250;
        delay(10);
    }
    
    blink(statusLED, 5, 500);   // tell user ready status
    
    Serial.print("Estimate offset (mV): " );
    Serial.println(offSet);
    
    digitalWrite(relaisK1, LOW);
    digitalWrite(relaisK2, LOW);
}

//
// Brief	Loop
//
// Add loop code 
void loop()
{
    measureCurrent();
    runMotor();
    measureDHT();
}
