//
// File			testLib.h
// Brief		Library header
//
// Project	 	rotationController
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author		Jonas Scharpf
// 				brainelectronics
// Date			07.10.15 18:47
// Version		<#version#>
//
// Copyright	© Jonas Scharpf, 2015
// License		<#license#>
//
// See			ReadMe.txt for references
//
// Core library - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430 G2 and F5529, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

#ifndef __rotationController__testLib__
#define __rotationController__testLib__
//
// Brief	Blink a LED
// Details	LED attached to pin is light on then light off
// Total cycle duration = ms
// Parameters:
//      pin pin to which the LED is attached
//      times number of times
//      ms cycle duration in ms
//
void blink2(uint8_t pin, uint8_t times, uint16_t ms);

#endif