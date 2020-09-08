/*
All hardware for Servotor32 is licensed under a Creative Commons Attribution ShareAlike 3.0 United States License.
The full terms of this license can be read at:
http://creativecommons.org/licenses/by-sa/3.0/

All software is licensed under the MIT License.
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef Servotor32_h
#define Servotor32_h

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#define STATUS_LED 7

#define SERVOS 32
#define MAX_TIMINGS 36

#define GROUPS 4
#define SERVOS_PER_GROUP 8


class Servotor32 {
public:
  Servotor32();
  void begin();
  
  long unsigned int micros_new();

  long unsigned int millis_new();
  void delay_ms(long unsigned int);
  void delay_us(long unsigned int);
  
//  void changeServo(byte, short);
  
  void printStatus(Stream*);
  
  void process(Stream*);
  
  float ping();
  float multiPing(unsigned short);

  void update_all_registers_fast();

  void sendToPC(int*, Stream*);
  void calibrateExtrema(Stream*, boolean);
  void printParameters(Stream*);
  void readSensorValues(Stream*);
  void positionAndSensor(Stream*);
  void positionFromSensor(Stream*);
  void stand();

 private:
  static void callback();
  short tallyCount();

};

#endif

