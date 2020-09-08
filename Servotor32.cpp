/*
All hardware for Servotor32 is licensed under a Creative Commons Attribution ShareAlike 3.0 United States License.
The full terms of this license can be read at:
http://creativecommons.org/licenses/by-sa/3.0/

All software is licensed under the MIT License.
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


// Hexapod locomotion via dynamical system starting from code line 315

#include "Servotor32.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

#include <math.h>

Servotor32::Servotor32() {

}
//stores information about the servos and groups
signed short servo_positions[SERVOS]; // where the servos are currently (supposed to be) at
signed char servos_sorted[GROUPS][SERVOS_PER_GROUP]; // index in servo_timings to where the servo ends
signed char servos_active_in_group[GROUPS]; // the number of servos in a group currently active
uint8_t active_servos_hex[GROUPS];

// all updates to shift registers in order of their updates
signed short servo_timings[MAX_TIMINGS]; // the timing where the change occurs
uint8_t shift_output[MAX_TIMINGS];  // the output of the shift register
uint8_t shift_latch[MAX_TIMINGS];   // the shift register latch used

// keeps track of whether its safe or not to update the servos
uint8_t update_reg_flag = 0;

// variables for the callback
uint16_t timer;
uint8_t counter = 0;
uint8_t pwm_active = 1;

uint16_t group_offsets[4] = {0, 251, 502, 753};
uint8_t group_latches[4] = {5, 6, 7, 4};
uint8_t pin_2_num[8] = {0x08, 0x04, 0x02, 0x01, 0x80, 0x40, 0x20, 0x10};

void Servotor32::begin() {
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs

  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);

  //setup PC serial port
  Serial.begin(9600);
  // reconfigure bluetooth module to 9600 baud id needed
  Serial1.begin(115200);     // Changed from 9600 baud
  Serial1.print("AT+BAUD4"); // Tell the module to change the baud rate to 9600
  delay(1100); // Wait a notch over 1 second to make sure the setting "sticks"
  Serial1.begin(9600);     // Changed from 9600 baud

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  Timer1.initialize(10);
  Timer1.attachInterrupt(callback);

  for (byte i = 0; i < SERVOS; i++) {
    servo_positions[i] = -1;
  }
  for (byte i = 0; i < GROUPS; i++) {
    for (byte j = 0; j < SERVOS_PER_GROUP; j++) {
      servos_sorted[i][j] = -1;
    }
  }

  for (uint8_t i = 0; i < MAX_TIMINGS; i++) {
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  TIMSK0 &= ~(_BV(TOIE0)); // disables the arduino delay function, but also
  // all but eliminates servo jitter
  TIMSK2 &= ~(_BV(TOIE2)); // disable the arduino tone  function, but also
  // also helps eliminate some jitter
  TIMSK3 &= ~(_BV(TOIE3)); // for good measure
  TIMSK4 &= ~(_BV(TOIE4)); // for good measure
}

long unsigned int us_counter = 0;
long unsigned int startTime = 0;
long unsigned int currentTime = 0;
long unsigned int last_update = 0;

long unsigned int Servotor32::micros_new() {
  return us_counter;
}

long unsigned int Servotor32::millis_new() {
  return us_counter / 1000;
}

void Servotor32::delay_ms(long unsigned int delay_time) {
  startTime = millis_new();
  currentTime = millis_new() - startTime;
  while (currentTime < delay_time) {
    delayMicroseconds(10);
    currentTime = millis_new() - startTime;
  }
}

void Servotor32::delay_us(long unsigned int delay_time) {
  startTime = micros_new();
  currentTime = micros_new() - startTime;
  while (currentTime < delay_time) {
    delayMicroseconds(10);
    currentTime = micros_new() - startTime;
  }
}

void Servotor32::callback() {
  cli();
  if (timer < 1100) { // keep it from updating servos mid-array change by some weird coincidence
    if (timer == servo_timings[counter]) { // if the time has arrived to update a shift reg
      SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
      while (!(SPSR & (1 << SPIF))); //wait till the register completes
      PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
      PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
      counter++;
    }
  }

  timer++;
  us_counter += 10;
  if (timer == 1100) { // all servo timing completed
    update_reg_flag = 1; // allow updates to the timing arrays
  }
  if (timer == 1900) { // getting close to servo start-up again,
    update_reg_flag = 0; // don't allow any new timing array updates
  }
  if (timer == 2000) {
    timer = 0;
    counter = 0;
  }
  sei();
}

// modify the state of a servo
void Servotor32::update_all_registers_fast() {
  while (update_reg_flag == 0) { // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- delete all ------
  for (byte group = 0; group < GROUPS; group++) {
    servos_active_in_group[group] = 0;
    active_servos_hex[group] = 0;
    for (byte i = 0; i < SERVOS_PER_GROUP; i++) {
      servos_sorted[group][i] = -1;
    }
  }

  // ----- add all active servos ------
  for (byte i = 0; i < SERVOS; i++) {
    if (servo_positions[i] != -1) {
      servos_sorted[i / SERVOS_PER_GROUP][servos_active_in_group[i / SERVOS_PER_GROUP]] = i; //index of active servo
      servos_active_in_group[i / SERVOS_PER_GROUP] += 1;
      active_servos_hex[i / SERVOS_PER_GROUP] |= pin_2_num[i % SERVOS_PER_GROUP];
    }
  }

  // ----- bubble sort servos_sorted ------
  // sort the array by servos position, the ones with pos = -1 will stay at the end
  boolean sorted = false;
  byte j = 0;
  short temp = 0;
  for (byte group = 0; group < GROUPS; group++) {   // for each group separately
    sorted = false;
    j = 0;
    while (!sorted) {   // continue sorting the array until is sorted.
      sorted = true;  // expecting the the array sorted
      for (byte i = 0; i < servos_active_in_group[group] - 1 - j;
           i++) {   // go through the active servo list (they are at the front)
        if (servo_positions[servos_sorted[group][i]]
            > servo_positions[servos_sorted[group][i + 1]]) {  //if wrong order of two consecutive, swap them
          temp = servos_sorted[group][i];
          servos_sorted[group][i] = servos_sorted[group][i + 1];
          servos_sorted[group][i + 1] = temp;
          sorted = false;   // no, it was not sorted yet
        }
      }
      j++;
    }
  }

  // ----- create timing idicies from servo/group data ------- (no change from here on)

  // clear the timing arrays for fresh start
  for (uint8_t i = 0; i < MAX_TIMINGS; i++) {
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index = 0;
  uint8_t current_timing = 0;
  uint8_t current_shift_output = 0;

  for (byte group = 0; group < GROUPS; group++) { //go through each group
    if (servos_active_in_group[group] > 0) { // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1 << group_latches[group]);
      counter_index += 1;


      //create additional timings
      for (byte i = 0; i < servos_active_in_group[group];
           i++) { //create the timings for each servo after that, using the previous output
        if (servo_positions[servos_sorted[group][i]]
            == servo_positions[servos_sorted[group][i - 1]]) { // if this servo's time is the same as the last's
          if (i != 0) {
            counter_index -= 1; //reverse the index count-up
          } else {
            current_shift_output = shift_output[counter_index - 1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]] + group_offsets[group];
            shift_latch[counter_index] = (1 << group_latches[group]);
          }
        } else {
          current_shift_output = shift_output[counter_index - 1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]] + group_offsets[group];
          shift_latch[counter_index] = (1 << group_latches[group]);
        }

        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i] - group * SERVOS_PER_GROUP];
        shift_output[counter_index] = current_shift_output;
        counter_index += 1;
      }
    }
  }

}

void Servotor32::printStatus(Stream *serial) {
//  serial->println("--------------------- Registers ----------------------");

  serial->println("Servo Data:");
  serial->println("Servo\tPos\tTimeEnd\t");
  for (byte i = 0; i < SERVOS; i++) {
    serial->print(i);
    serial->print("\t");
    serial->print(servo_positions[i]);
    serial->println("");
  }
  serial->println("");

  serial->println("Sorted Groups");
  for (byte i = 0; i < GROUPS; i++) {
    serial->print("Group: ");
    serial->println(i);
    for (byte j = 0; j < SERVOS_PER_GROUP; j++) {
      serial->print("Servo: ");
      serial->print(servos_sorted[i][j]);
      serial->print("\t");
      serial->println(servo_positions[servos_sorted[i][j]]);

    }
  }

  serial->println("Group Data:");
  serial->println("#\tActive\tHex");
  for (byte i = 0; i < GROUPS; i++) {
    serial->print(i);
    serial->print("\t");
    serial->print(servos_active_in_group[i]);
    serial->print("\t");
    serial->println(active_servos_hex[i], HEX);
  }
  serial->println("");

  serial->println("Timings:");
  serial->println("Pos\tTiming\tOutput\tLatch");
  for (uint8_t i = 0; i < MAX_TIMINGS; i++) { // clear existing registers, so they can be cleanly written
    serial->print(i);
    serial->print(":\t");
    serial->print(servo_timings[i]);
    serial->print(",\t");
    serial->print(shift_output[i], HEX);
    serial->print(",\t");
    serial->println(shift_latch[i], HEX);
  }
//  serial->println("----------------------------------------------------");
}

// booleans for switch statements
boolean servoCounting = false;
boolean posCounting = false;
boolean localCalc = false;

byte numString[6];
int powers[] = {1, 10, 100, 1000};

byte numCount = 0;
unsigned short total = 0;
short inServo = -1;
short inPos = -1;

//int analogSensors_arr[8] = {A4, A5, A6, A7, A8, A9, A10, A11};
int analogSensors_arr[6] = {A4, A5, A6, A7, A8, A9}; // variables for analog sensor pins

// leg numeration: 0: left front, 1: left middle, 2: left rear, 3: right rear, 4: right middle, 5: right front
// servo numbers (numbered pins on Servotor32 board)
byte vertical_leg_num[6] = {6, 10, 14, 17, 21, 25};
byte horizontal_leg_num[6] = {7, 11, 15, 16, 20, 24};
byte tibia_leg_num[6] = {5, 9, 13, 18, 22, 26};

// motor positions and corresponding sensor values of motion range (minimum, maximum)
// vertical
//byte vertical_min[6] = {211, 222, 218, 217, 223, 223}; // physically minimal vertical motor positions
byte vertical_min[6] = {181, 192, 188, 187, 193, 193}; // physically minimal vertical motor positions
int sensor_vert_min[6] = {258, 222, 221, 220, 206, 203}; // minimal vertical sensor values

byte vertical_max[6] = {120, 135, 135, 130, 135, 135}; // maximal vertical motor positions
int sensor_vert_max[6] = {446, 392, 394, 390, 383, 377}; // maximal vertical sensor values

// horizontal
byte horizontal_min[6] = {190, 177, 150, 157, 129, 120}; // minimal horizontal motor positions
int sensor_horiz_min[6] = {239, 292, 348, 315, 409, 448}; // minimal horizontal sensor values

byte horizontal_max[6] = {155, 125, 125, 185, 180, 154}; // maximal horizontal motor positions
int sensor_horiz_max[6] = {341, 421, 437, 236, 253, 332}; // maximal horizontal sensor values

// tibia (no sensor values needed)
//byte tibia_min[6] = {60, 80, 55, 80, 80, 60};  // rectangluar tibia motor position to femur
byte tibia_min[6] = {105, 110, 97, 115, 110, 107};  // minimum tibia motor position for stand
byte tibia_max[6] = {130, 135, 135, 145, 140, 140}; // maximal tibia motor position

byte legsets[3] = {2, 3, 6}; // number of main phase shifts
byte legpairs[3] = {3, 2, 1}; // number of oscillators in phase

// gait leg sequences:
byte tripod_seq[6] = {1, 2, 1, 2, 1, 2}; // {0,2,4} alternating with {1,3,5}
byte tetrapod_seq[6] = {1, 2, 3, 1, 3, 2}; // {0,3}, {1,5} and {2,4} alternating
byte wave_seq[6] = {1, 2, 3, 4, 5, 6}; // 0-5 individual in sequence
byte *gait_seq_arr[3] = {tripod_seq, tetrapod_seq, wave_seq};

// dynamical variables
float membrane[6]; // vertical neural membrane potential
float bias_vert[6]; // vertical neural bias

float analog_sensor_norm_real[6]; // normalized sensor read
float analog_sensor_norm[6];  // normalized sensor read with possible cutoff exceeding [0,1]


// accessory variables
float matrix[6][6]; // gait matrix

float neuron_vert[6]; // vertical neural activation
float feedback[6]; // membrane potential unscaled feedback input
float neuron_horiz[6]; // horizontal neural activation
float gain_horiz[6]; // horizontal neural gain
float bias_vert_sens_horiz[6]; // vertical neural bias from horizontal sensor

float bias_vert_min[6]; // bias minimum of current period
float bias_vert_max[6]; // bias maximum of current period
float bias_vert_avg[6]; // bias average of current period

float bias_vert_old[6]; // vertical neural bias from previous iteration step
float bias_vert_diff_old[6]; // bias difference in previous iteration step

float direction[6] = {1, 1, 1, 1, 1, 1}; // direction array for individual leg moving direction

// auxiliary variables
byte pos_min[6];
byte pos_max[6];
int sensor_max[6];
int sensor_min[6];
int sensor_values[6];

const byte inputCount = 22; // number of observables to write to serial
int input_array[inputCount]; // array of observables

// time variables
unsigned long time_null = 0;
unsigned long while_start = 0;
int target_dt_millis = 105;
int dt_millis = 105;
int delay_dt = 0;
float dt = 0.105;
unsigned int time_total = 0;

int step_count = 0;

boolean sendData = true; // write data to serial port
boolean calibrate = true; // calibrate sensor values to position values for min-max normalization
boolean sendToMotors = true; // send calculation results to motors
boolean read_sensor = true; // read motor sensor values
boolean test_print = false; // continuously prints set variable values
boolean sensor_cutoff = false; // restrict sensor values to [0,1], cuts off values exceeding the limits
boolean tripod_calibration = false; // calibrate the sensor values to the motor limits with three legs on the ground
                                  // otherwise calibration in air

byte locomotion = 0; // 0: default, 1: stomp, 2: stand
char rangeshift = 0; // +- motor position correction for horizontal motor position range

byte sensor_start = 5; // iteration step for closing the feedback loop; ensures stability at start

float ics_tau = 0.1; // time shift in seconds for neural bias initial conditions
float b_dot_bin; // time derivative of neural bias with binary approximation for initial conditions

byte sensors = 1; // 0: vertical, 1: horizontal
byte feedback_loop = 1; // 0: open, 1: closed

// gait settings
byte gait_num = 0; // gait number; 0: tripod, 1: tetrapod, 2: wave

// parameter arrays
float gain_vert_arr[3] = {5., 5., 5.}; // neural gain
float tau_x_arr[3] = {0.13, 0.13, 0.13}; // time constant of membrane potential
float tau_b_arr[3] = {0.2, 0.2, 0.2}; // time constant of neural bias
float weight_arr[3] = {1.7, 3, 7}; // scaling factor of membrane potential feedback input
float neuron_avg_arr[3] = {0.4, 0.27, 0.13}; // neural activation mean y_bar

float neuron_horiz_max = 0.95; // symmetrically restrict horizontal motion range to this fraction

// auxiliary parameter variables
float gain_vert = gain_vert_arr[gait_num];
float tau_b = tau_b_arr[gait_num];
float tau_x = tau_x_arr[gait_num];
float weight = weight_arr[gait_num];
float neuron_avg = neuron_avg_arr[gait_num];

void Servotor32::process(Stream *serial) { // called in Servotor32_Firmware.ino as 'hexy' object member function
  // with serial address
  if (serial->available()) { // process input from defined serial port
    char inChar = (char) serial->read(); // read serial input into inChar
    if (localCalc) {
      switch (inChar) {
        case '\r':
        case '\n':numCount = 0;
          localCalc = false;

          time_total = 0; // reinitialize for reentering this switch case
          step_count = 0; // reinitialize for reentering this switch case

          // initial conditions

          // gait matrix calculation based on given leg numeration and moving sequence
          for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
              matrix[i][j] = 1 - 2 * (float) ((gait_seq_arr[gait_num][j] + (legsets[gait_num]
                  - gait_seq_arr[gait_num][i])) % legsets[gait_num]) / (legsets[gait_num] - 1);
            }
          }

          // dynamical variables
          for (int i = 0; i < 6; i++) {
            // membrane potential ics
            membrane[i] = legpairs[gait_num] * weight * matrix[i][0]; // here column is used or transposed matrix row

            // neural bias ics with respect to membrane potential ics
            // time shifted by ics_tau with linear bias slope b_dot_bin
            if (matrix[i][0] == 1) { // with first matrix column the entries with 1 have a different shift
              b_dot_bin = (1 - neuron_avg) / tau_b; // \dot{b} where y=1
            } else {
              b_dot_bin = -neuron_avg / tau_b; // \dot{b} where y=0
            }
            // ics bias values between 0 and legpairs[gait_num] * weight
            bias_vert[i] = legpairs[gait_num] * weight * (matrix[i][0] + 1) / 2 - b_dot_bin * ics_tau;

            // additional ics
            bias_vert_old[i] = legpairs[gait_num] * weight * (matrix[i][0] + 1) / 2 - b_dot_bin * (ics_tau
                + target_dt_millis / 1000.);
            bias_vert_diff_old[i] = bias_vert[i] - bias_vert_old[i];
            bias_vert_min[i] = 0;
            bias_vert_max[i] = legpairs[gait_num] * weight;
            bias_vert_avg[i] = (bias_vert_max[i] + bias_vert_min[i]) / 2;
            bias_vert_sens_horiz[i] = bias_vert[i];
            neuron_vert[i] = 1 / (1 + exp(gain_vert * (bias_vert[i] - membrane[i])));
            gain_horiz[i] = 2 * log(1 / neuron_horiz_max - 1) /
                (bias_vert_min[i] - bias_vert_max[i]);
            neuron_horiz[i] = 1 / (1 + exp(direction[i] * gain_horiz[i] * (bias_vert_avg[i] - bias_vert[i])));
          }

          // course correction for straight forward walking
          if (calibrate) { // bool due to needed sensor calibration for closed loop
            for (int i = 0; i < 3; i++) {
              horizontal_min[i] -= rangeshift;
              horizontal_max[i] -= rangeshift;
            }
            for (int i = 3; i < 6; i++) {
              horizontal_min[i] += rangeshift;
              horizontal_max[i] += rangeshift;
            }
          }

          if (calibrate == false) { // use stored sensor values if calibration function is not called
            if (sensors == 0) { // for vertical sensors
              for (int i = 0; i < 6; i++) {
                sensor_min[i] = sensor_vert_min[i];
                sensor_max[i] = sensor_vert_max[i];
              }
            } else if (sensors == 1) { // for horizontal sensors
              for (int i = 0; i < 6; i++) {
                sensor_min[i] = sensor_horiz_min[i];
                sensor_max[i] = sensor_horiz_max[i];
              }
            }
          }

          if (sendToMotors) { // accessed if motors should be moved
            stand();
            // calibration
            if (calibrate) {
              calibrateExtrema(serial, false); // calibrating sensor values to position values for min-max normalization
            }
            stand();
            for (int i = 0; i < 6; i++) {
              sensor_values[i] = analogRead(analogSensors_arr[i]); // storing read sensor values in array
              analog_sensor_norm_real[i] = (float) (sensor_values[i] - sensor_min[i]) / (sensor_max[i] - sensor_min[i]);
            }
          }


          // waiting after calibration to bring robot in running position until start signal
          while (true) {  // waiting until '$' is received via serial port
            delayMicroseconds(10);
            inChar = (char) serial->read();  //read serial input every step
            if (inChar == '$') {
              break;
            }
          }

          // initial values (storing and sending)
          if (sendData) { //int input variables for serial streaming
            input_array[0] = -1; // step
            input_array[1] = -100; // total time in milliseconds
            input_array[2] = 0; // integration time step
            input_array[3] = 0; // delay time
            for (int i = 0; i < 6; i++) {
              input_array[i + 4] = round(membrane[i] * 1000); // neural membrane potential
              input_array[i + 10] = round(bias_vert[i] * 1000); // neural bias
              input_array[i + 16] = round(analog_sensor_norm_real[i] * 1000); // normalized sensor values

//              input_array[i + 22] = round(neuron_vert[i] * 1000); // vertical neural activation
//              input_array[i + 28] = round(feedback[i] * 1000); // membrane potential unscaled feedback input
//              input_array[i + 34] = round(neuron_horiz[i] * 1000); // horizontal neural activation
//              input_array[i + 40] = round(gain_horiz[i] * 1000); // horizontal neural gain
//              input_array[i + 46] = round(bias_vert_sens_horiz[i] * 1000); // horizontal neural bias
//
//              input_array[i + 52] = round(bias_vert_min[i] * 1000); // bias minimum of current period
//              input_array[i + 58] = round(bias_vert_max[i] * 1000); // bias maximum of current period
//              input_array[i + 64] = round(bias_vert_avg[i] * 1000); // bias average of current period
            }
            //sending buffered data via serial
            sendToPC(input_array, serial);
          }

          while_start = millis_new(); // total runtime initialization

          while (inChar != '%') {  //calculation runs until '%' is received via serial port
            time_null = millis_new();  // time initialization to measure iteration time

            // controls
            inChar = (char) serial->read();  //read serial input every step
            if (serial->available() != 0) {  // change parameter on the run
              switch (inChar) {
                case 'G':weight += 0.1;
                  break;
                case 'g':weight -= 0.1;
                  break;
                case 'A':gain_vert += 0.1;
                  break;
                case 'a':gain_vert -= 0.1;
                  break;
                case 'B':tau_b += 0.01;
                  break;
                case 'b':tau_b -= 0.01;
                  break;
                case 'X':tau_x += 0.01;
                  break;
                case 'x':tau_x -= 0.01;
                  break;
                case 'N':neuron_avg += 0.025;
                  break;
                case 'n':neuron_avg -= 0.025;
                  break;
                case 0:goto locomotion_type; // run
                  break;
                case 1:goto locomotion_type; // stomp
                  break;
                case 2:goto locomotion_type; // stand
                  break;
                locomotion_type:
                  locomotion = inChar;
                  break;
                case 'R':
                  for (int i = 0; i < 6; i++) {
                    direction[i] *= -1;
                  }
                  break;
                case 'd':
                  for (int i = 0; i < 3; i++) {
                    direction[i] *= -1;
                  }
                  break;
                case 'D':
                  for (int i = 3; i < 6; i++) {
                    direction[i] *= -1;
                  }
                  break;
                case 'S':rangeshift += 5; // only for open loop, due to sensor calibration
                  goto new_range;
                  break;
                case 's':rangeshift -= 5;
                  goto new_range;
                  break;
                new_range:
                  for (int i = 0; i < 3; i++) {
                    horizontal_min[i] -= rangeshift;
                    horizontal_max[i] -= rangeshift;
                  }
                  for (int i = 3; i < 6; i++) {
                    horizontal_min[i] += rangeshift;
                    horizontal_max[i] += rangeshift;
                  }
                  break;
                case 't':neuron_avg = neuron_avg_arr[0];
                  break;
                case 'q':neuron_avg = neuron_avg_arr[1];
                  break;
                case 'w':neuron_avg = neuron_avg_arr[2];
                  break;
                case 'T':gait_num = 0;
                  goto gaitCalc;
                  break;
                case 'Q':gait_num = 1;
                  goto gaitCalc;
                  break;
                case 'W':gait_num = 2;
                  goto gaitCalc;
                  break;
                gaitCalc:
                  weight = weight_arr[gait_num]; // use initialized gait specific weight values
                  neuron_avg = neuron_avg_arr[gait_num]; // use initialized gait specific neruon_avg values
                  for (int i = 0; i < 6; i++) {
                    for (int j = 0; j < 6; j++) { // gait matrix calculation
                      matrix[i][j] = 1 - 2
                          * ((gait_seq_arr[gait_num][j] + (legsets[gait_num] - gait_seq_arr[gait_num][i]))
                              % legsets[gait_num]) / (legsets[gait_num] - 1);
                    }
                  }
                  break;
                case 'i':serial->println(String()); // leaving this out gives an error
                  printParameters(serial); // print the set parameters to serial
                  break;
              }
            }

            // sensor
            if (read_sensor) { // reading analog sensor values
              //analogRead() reads out the pins A4-A11 where the encoders of the motors are plugged in
              for (int i = 0; i < 6; i++) {
                sensor_values[i] = analogRead(analogSensors_arr[i]); // storing read sensor values in array
              }

              for (int i = 0; i < 6; i++) { // min-max normalization of sensor values with distinction for cutoff
                analog_sensor_norm_real[i] = (float) (sensor_values[i] - sensor_min[i]) / (sensor_max[i] - sensor_min[i]);
                analog_sensor_norm[i] = analog_sensor_norm_real[i];
              }

              if (sensor_cutoff) { // exceeding the extrema should not be possible
                // this partly filters fluctuations due to cutoff value, if wanted
                for (int i = 0; i < 6; i++) {
                  if (analog_sensor_norm_real[i] > 1) {
                    analog_sensor_norm[i] = 1;
                  } else if (analog_sensor_norm_real[i] < 0) {
                    analog_sensor_norm[i] = 0;
                  }
                }
              }
            }

            // membrane potential feedback
            if ((sensors == 0) && (feedback_loop == 1) && (step_count > sensor_start)) {
              // closed loop with vertical sensor feedback
              for (int i = 0; i < 6; i++) {
                feedback[i] = 0.;  //reset feedback input
                for (int j = 0; j < 6; j++) {
                  feedback[i] += matrix[i][j] * analog_sensor_norm[j];
                }
              }
            } else { // default feedback calculation
              for (int i = 0; i < 6; i++) {
                feedback[i] = 0.;  //reset feedback input
                for (int j = 0; j < 6; j++) {
                  feedback[i] += matrix[i][j] * neuron_vert[j];
                }
              }
            }

            // neural membrane potential
            for (int i = 0; i < 6; i++) { // default feedback calculation
              membrane[i] += 1. / tau_x * (weight * feedback[i] - membrane[i]) * dt;
            }

            // neural activation
            if ((sensors == 1) && (feedback_loop == 1) && (step_count > sensor_start)) {
              // closed loop with horizontal sensor feedback
              // sensor start delays feeedback input
              for (int i = 0; i < 6; i++) {
                bias_vert_sens_horiz[i] = bias_vert_avg[i] + direction[i] * ((1 - analog_sensor_norm[i]) * (bias_vert_min[i]
                    - bias_vert_avg[i]) + analog_sensor_norm[i] * (bias_vert_max[i] - bias_vert_avg[i]));
                neuron_vert[i] = 1 / (1 + exp(gain_vert * (bias_vert_sens_horiz[i] - membrane[i])));
              }
            } else { // default vertical neural activation calculation
              for (int i = 0; i < 6; i++) {
                neuron_vert[i] = 1 / (1 + exp(gain_vert * (bias_vert[i] - membrane[i])));
              }
            }

            // neural bias
            if ((sensors == 0) && (feedback_loop == 1) && (step_count > sensor_start)) {
              // closed loop with vertical sensor feedback
              // sensor_start delays feedback input
              for (int i = 0; i < 6; i++) {
                bias_vert[i] += 1. / tau_b * (analog_sensor_norm[i] - neuron_avg) * dt;
              }
            } else { // default vertical neural bias calculation
              for (int i = 0; i < 6; i++) {
                bias_vert[i] += 1. / tau_b * (neuron_vert[i] - neuron_avg) * dt;
              }
            }

            // determine extrema of vertical neuron's bias (b_min, b_max) of current period
            for (int i = 0; i < 6; i++) {
              if (((bias_vert[i] - bias_vert_old[i]) < 0)
                  && ((bias_vert[i] - bias_vert_old[i]) * bias_vert_diff_old[i] < 0)) {
                bias_vert_max[i] = bias_vert_old[i];
              } else if (((bias_vert[i] - bias_vert_old[i]) > 0)
                  && ((bias_vert[i] - bias_vert_old[i]) * bias_vert_diff_old[i] < 0)) {
                bias_vert_min[i] = bias_vert_old[i];
              }
              if (bias_vert[i] > bias_vert_max[i]) {
                bias_vert_max[i] = bias_vert[i];
              } else if (bias_vert[i] < bias_vert_min[i]) {
                bias_vert_min[i] = bias_vert[i];
              }
              bias_vert_avg[i] = (bias_vert_max[i] + bias_vert_min[i]) / 2; // average bias for current period

              // store data to use as previous calculation step
              bias_vert_diff_old[i] = bias_vert[i] - bias_vert_old[i];
              bias_vert_old[i] = bias_vert[i];

              // adaptive gain
              gain_horiz[i] = 2 * log(1 / neuron_horiz_max - 1) / (bias_vert_min[i] - bias_vert_max[i]);
              // horizontal neural activation
              neuron_horiz[i] = 1 / (1 + exp(direction[i] * gain_horiz[i] * (bias_vert_avg[i] - bias_vert[i])));
            }

            // send calculation results to motor
            if (sendToMotors) {
              switch (locomotion) { //  storing next motor positions in array
                // stomp at position
                case 1: // horizontal motor positions fixed, while vertical motion continues; preferably with
                  // open loop or vertical closed loop
                  for (int i = 0; i < 6; i++) {
                    servo_positions[vertical_leg_num[i]] =
                        round((1 - neuron_vert[i]) * vertical_min[i] + neuron_vert[i] * vertical_max[i]);
                    servo_positions[horizontal_leg_num[i]] = round((horizontal_max[i] + horizontal_min[i]) / 2.);
//                    servo_positions[tibia_leg_num[i]] = round((1 - neuron_vert[i]) * tibia_min[i]
//                        + neuron_vert[i] * tibia_max[i]);
                  }
                  // stand
                case 2: // motor positions fixed, but calculation continues; preferably with open loop;
                  // can be used for switching gaits until stable limit cycle is reached
                  for (int i = 0; i < 6; i++) {
                    servo_positions[vertical_leg_num[i]] = vertical_min[i];
                    servo_positions[horizontal_leg_num[i]] = round((horizontal_max[i] + horizontal_min[i]) / 2.);
//                    servo_positions[tibia_leg_num[i]] = tibia_min[i];
                  }
                  // run
                case 0: // default motor positions
                  for (int i = 0; i < 6; i++) {
                    servo_positions[vertical_leg_num[i]] = round((1 - neuron_vert[i]) * vertical_min[i]
                                                                     + neuron_vert[i] * vertical_max[i]);
                    servo_positions[horizontal_leg_num[i]] = round((1 - neuron_horiz[i]) * horizontal_min[i]
                                                                       + neuron_horiz[i] * horizontal_max[i]);
//                    servo_positions[tibia_leg_num[i]] = round((1 - neuron_vert[i]) * tibia_min[i]
//                        + neuron_vert[i] * tibia_max[i]);
                  }
              }
              update_all_registers_fast();  // update motor positions
            }

            // send calculation results to PC
            if (sendData) { //int input for serial streaming
              input_array[0] = step_count;
              input_array[1] = time_total;
              input_array[2] = dt_millis;
              input_array[3] = delay_dt;
              for (int i = 0; i < 6; i++) {
                input_array[i + 4] = round(membrane[i] * 1000);
                input_array[i + 10] = round(bias_vert[i] * 1000);
                input_array[i + 16] = round(analog_sensor_norm_real[i] * 1000);

//                input_array[i + 22] = round(neuron_vert[i] * 1000); // vertical neural activation
//                input_array[i + 28] = round(feedback[i] * 1000); // membrane potential unscaled feedback input
//                input_array[i + 34] = round(neuron_horiz[i] * 1000); // horizontal neural activation
//                input_array[i + 40] = round(gain_horiz[i] * 1000); // horizontal neural gain
//                input_array[i + 46] = round(bias_vert_sens_horiz[i] * 1000); // horizontal neural bias
//
//                input_array[i + 52] = round(bias_vert_min[i] * 1000); // bias minimum of current period
//                input_array[i + 58] = round(bias_vert_max[i] * 1000); // bias maximum of current period
//                input_array[i + 64] = round(bias_vert_avg[i] * 1000); // bias average of current period
              }
              sendToPC(input_array, serial); // sending buffered data via serial
            }

            // real time integration step size
            dt_millis = millis_new() - time_null; // determining calculation time of iteration in milliseconds
            delay_dt = target_dt_millis - dt_millis; // determining iteration wait time on set calculation step size
            while (dt_millis < target_dt_millis) {
              delayMicroseconds(10);
              dt_millis = millis_new() - time_null;
            }

            dt = dt_millis * 0.001; // setting step size for next interation
            time_total = millis_new() - while_start; // tracking runtime

            // Variable test print
            if ((test_print == true) && (sendData == false)) { // sending buffered data can interfere with printing
              if (step_count % 10 == 0) {
                serial->println(delay_dt);
              }
            }

            step_count++; // iteration count

          } // end of iteration loop
          break;
      } // end of switch (inChar)
    } else {
      switch (inChar) {
        case '&':  //starts local calculation
          localCalc = true;
          numCount = 0;
          break;
        case '#':servoCounting = true;
          numCount = 0;
          inServo = -1;
          inPos = -1;
          break;
        case 'D':printStatus(serial);
          break;
        case 'P':
          if (servoCounting) {
            inServo = tallyCount();
            servoCounting = false;
          }
          posCounting = true;
          numCount = 0;
          break;
        case '\r':
        case '\n':
          if (posCounting) {
            inPos = tallyCount();
            posCounting = false;
          }
          if ((inServo >= 0) && (inServo <= 31) && (((inPos >= 50) && (inPos <= 250)) || (inPos == -1))) {
            servo_positions[inServo] = inPos;
            update_all_registers_fast();
            serial->print('#');
            serial->print(inServo);
            serial->print('P');
            serial->println(inPos);
            inServo = -1;
            inPos = -1;
          }
          numCount = 0;
          break;
//        case 'V':serial->println("SERVOTOR32_v2.0mod");
//          break;
        case 'C':
          for (int i = 0; i < 6; i++) {
            servo_positions[horizontal_leg_num[i]] = 150;
            servo_positions[vertical_leg_num[i]] = 150;
            servo_positions[tibia_leg_num[i]] = 150;
          }
          update_all_registers_fast();
          delay_ms(1100);
//          serial->println("All Centered");
          break;
        case 'S': //walking pose
          stand();
//          serial->println("Ready to run");
          break;
        case 'F': //pose for transport
          for (int i = 0; i < 6; i++) {
            servo_positions[horizontal_leg_num[i]] = (horizontal_max[i] + horizontal_min[i]) / 2;
            servo_positions[vertical_leg_num[i]] = 80;
            servo_positions[tibia_leg_num[i]] = 80;
          }
          servo_positions[tibia_leg_num[0]] = 70;
          update_all_registers_fast();
          delay_ms(1100);
          servo_positions[horizontal_leg_num[0]] = 60;
          servo_positions[horizontal_leg_num[1]] = 80;
          servo_positions[horizontal_leg_num[2]] = 90;
          servo_positions[horizontal_leg_num[3]] = 200;
          servo_positions[horizontal_leg_num[4]] = 220;
          servo_positions[horizontal_leg_num[5]] = 240;
          update_all_registers_fast();
          delay_ms(1100);
          for (int i = 0; i < 6; i++) {
            servo_positions[horizontal_leg_num[i]] = -1;
            servo_positions[vertical_leg_num[i]] = -1;
            servo_positions[tibia_leg_num[i]] = -1;
          }
          update_all_registers_fast();
          delay_ms(1100);
//          serial->println("All Turned Off");
//          serial->println("Ready to pack");
          break;
        case 'N':
          for (int i = 0; i < 6; i++) { // minimal horizontal motion range
            servo_positions[horizontal_leg_num[i]] = horizontal_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          goto killall;
          break;
        case 'X':
          for (int i = 0; i < 6; i++) { // maximal horizontal motion range
            servo_positions[horizontal_leg_num[i]] = horizontal_max[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          goto killall;
          break;
        case 'M':
          for (int i = 0; i < 6; i++) { // minimal vertical motion range
            servo_positions[vertical_leg_num[i]] = vertical_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          goto killall;
          break;
        case 'Y':
          for (int i = 0; i < 6; i++) { // maximal vertical motion range
            servo_positions[vertical_leg_num[i]] = vertical_max[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          goto killall;
          break;
        case 't':
          for (int i = 0; i < 6; i++) { // tibia position for stand
            servo_positions[tibia_leg_num[i]] = tibia_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          goto killall;
          break;
        case 'K':
        killall:
          step_count = 0;
          time_total = 0;
          for (int i = 0; i < 6; i++) {
            servo_positions[horizontal_leg_num[i]] = -1;
            servo_positions[vertical_leg_num[i]] = -1;
            servo_positions[tibia_leg_num[i]] = -1;
          }
          update_all_registers_fast();
          delay_ms(1100);
//          serial->println("All Turned Off");
          break;
        case 'i':printParameters(serial);
          break;
        case 'R':readSensorValues(serial);
          break;
        case 'r':positionAndSensor(serial);
          break;
        case 'e':calibrateExtrema(serial, true);
          goto killall;
          break;
        case 'E':positionFromSensor(serial);
          break;
//        case 'L':
//          if (servoCounting) {
//            inServo = tallyCount();
//            servoCounting = false;
//          }
//          servo_positions[inServo] = -1;
//          update_all_registers_fast();
//          break;
        default:
          if ((inChar > 47) && (inChar < 58)) {
            if (numCount < 4) {
              numString[numCount] = inChar - 48;
              numCount++;
            }
          }
          break;
      }
    }
  }
}

void Servotor32::printParameters(Stream *serial) { // print the current set parameters
  serial->print("g=");
  serial->print(weight);
  serial->print(", ");
  serial->print("y_bar=");
  serial->print(neuron_avg);
  serial->print(", ");
  serial->print("a=");
  serial->print(gain_vert);
  serial->print(", ");
  serial->print("tau_b=");
  serial->print(tau_b);
  serial->print(", ");
  serial->print("tau_x=");
  serial->print(tau_x);
  serial->print(", ");
  serial->print("Y_max=");
  serial->print(neuron_horiz_max);
  serial->print(", ");
  serial->print("gait=");
  serial->print(gait_num);
  serial->print(", ");
  serial->print("loop=");
  serial->print(feedback_loop);
  serial->print(", ");
  serial->print("sensors=");
  serial->print(sensors);
  serial->print(", ");
  serial->print("sensor_start=");
  serial->print(sensor_start);
  serial->print(", ");
  serial->print("direction=");
  serial->print(direction[0]);
  serial->print(", ");
  serial->print("dt=");
  serial->print(target_dt_millis);
  serial->print(", ");
  serial->print("ics_tau=");
  serial->print(ics_tau);
  serial->print(", ");
  serial->print("sensor_cutoff=");
  serial->print(sensor_cutoff);
  serial->print(", ");
  serial->print("tripod_calibration=");
  serial->print(tripod_calibration);
  serial->println("");
}

void Servotor32::stand() { // motor positions for steady stable stand
  servo_positions[horizontal_leg_num[0]] = horizontal_max[0];
  servo_positions[horizontal_leg_num[1]] = (horizontal_max[1] + horizontal_min[1]) / 2;
  servo_positions[horizontal_leg_num[2]] = horizontal_min[2];
  servo_positions[horizontal_leg_num[3]] = horizontal_min[3];
  servo_positions[horizontal_leg_num[4]] = (horizontal_max[4] + horizontal_min[4]) / 2;
  servo_positions[horizontal_leg_num[5]] = horizontal_max[5];

  for (int i = 0; i < 6; i++) {
    servo_positions[vertical_leg_num[i]] = vertical_min[i];
    servo_positions[tibia_leg_num[i]] = tibia_min[i];
  }
  update_all_registers_fast();
  delay_ms(1100);
}

void Servotor32::readSensorValues(Stream *serial) { // print sensor values from current motor positions
  for (int i = 0; i < 6; i++) {
    sensor_values[i] = analogRead(analogSensors_arr[i]);
    serial->print("sensor ");
    serial->print(i);
    serial->print("\t");
    serial->println(sensor_values[i]);
  }
  serial->print("{");
  for (int i = 0; i < 6; i++) {
    serial->print(sensor_values[i]);
    if (i < 5) {
      serial->print(", ");
    }
  }
  serial->println("}");
}

void Servotor32::positionFromSensor(Stream *serial) { // inter and extrapolate motor position values linearly from
  // min and max read sensor values for manual leg calibration
  if (sensors == 0) {
    for (int i = 0; i < 6; i++) {
      sensor_min[i] = sensor_vert_min[i];
      sensor_max[i] = sensor_vert_max[i];
      pos_min[i] = vertical_min[i];
      pos_max[i] = vertical_max[i];
    }
  } else if (sensors == 1) {
    for (int i = 0; i < 6; i++) {
      sensor_min[i] = sensor_horiz_min[i];
      sensor_max[i] = sensor_horiz_max[i];
      pos_min[i] = horizontal_min[i];
      pos_max[i] = horizontal_max[i];
    }
  }
  int off_min[6];
  int off_max[6];
  int off_avg[6];
  float slope[6];
  int pos[6];
  for (int i = 0; i < 6; i++) {
    sensor_values[i] = analogRead(analogSensors_arr[i]);
    off_min[i] = -((float) (pos_min[i] - pos_max[i]) / (sensor_min[i] - sensor_max[i]) * sensor_min[i] - pos_min[i]);
    off_max[i] = -((float) (pos_min[i] - pos_max[i]) / (sensor_min[i] - sensor_max[i]) * sensor_max[i] - pos_max[i]);
    off_avg[i] = (float) (off_min[i] + off_max[i]) / 2;
    slope[i] = (float) (pos_min[i] - pos_max[i]) / (sensor_min[i] - sensor_max[i]);
    pos[i] = slope[i] * (float) sensor_values[i] + (float) off_avg[i];
    serial->print(sensor_values[i]);
    serial->print("\t");
    serial->print(pos[i]);
    serial->print("\t");
    serial->print(slope[i]);
    serial->print("\t");
    serial->println(off_avg[i]);
  }
  serial->println("");
  serial->print("sensor={");
  for (int i = 0; i < 6; i++) {
    serial->print(sensor_values[i]);
    if (i < 5) {
      serial->print(", ");
    }
  }
  serial->print("};");
  serial->print("\t");
  serial->print("position={");
  for (int i = 0; i < 6; i++) {
    serial->print(pos[i]);
    if (i < 5) {
      serial->print(", ");
    }
  }
  serial->println("}");
}

void Servotor32::positionAndSensor(Stream *serial) { // go stepwise within motion range and print sensor values
  // to check for sensor value linearity
  if (sensors == 0) {
    for (int i = 0; i < 6; i++) {
      servo_positions[vertical_leg_num[i]] = vertical_min[i];
    }
  } else if (sensors == 1) {
    for (int i = 0; i < 6; i++) {
      servo_positions[horizontal_leg_num[i]] = horizontal_min[i];
    }
  }
  update_all_registers_fast();
  delay_ms(1100);

  if (sensors == 0) {
    for (int i = 0; i < 6; i++) {
      pos_min[i] = vertical_min[i];
      pos_max[i] = vertical_max[i];
    }
  } else if (sensors == 1) {
    for (int i = 0; i < 6; i++) {
      pos_min[i] = horizontal_min[i];
      pos_max[i] = horizontal_max[i];
    }
  }

  byte pos[6];
  byte steps = 10;
  char extend = 4;
  char k = -1 * extend;

  while (k < steps + extend) {
    for (int i = 0; i < 6; i++) {
      pos[i] = pos_min[i] + k * (pos_max[i] - pos_min[i]) / steps;
    }
    if (sensors == 0) {
      for (int i = 0; i < 6; i++) {
        servo_positions[vertical_leg_num[i]] = pos[i];
      }
    } else if (sensors == 1) {
      for (int i = 0; i < 6; i++) {
        servo_positions[horizontal_leg_num[i]] = pos[i];
      }
    }
    k++;
    update_all_registers_fast();
    delay_ms(500);
    serial->println("");
    serial->print("sensor={");
    for (int i = 0; i < 6; i++) {
      serial->print(analogRead(analogSensors_arr[i]));
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->print("};");
    serial->print("\t");
    serial->print("position={");
    for (int i = 0; i < 6; i++) {
      serial->print(pos[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->print("}");
    delay_ms(500);
  }
}

void Servotor32::calibrateExtrema(Stream *serial, boolean printSensorLimits) { // store sensor values of given motor positions extrema
  if (tripod_calibration == false) {
    if (sensors == 0) {
      for (int i = 0; i < 6; i++) {
        servo_positions[vertical_leg_num[i]] = vertical_min[i];
      }
    } else if (sensors == 1) {
      for (int i = 0; i < 6; i++) {
        servo_positions[horizontal_leg_num[i]] = horizontal_min[i];
      }
    }
    update_all_registers_fast();
    delay_ms(600);
    for (int i = 0; i < 6; i++) {
      sensor_min[i] = analogRead(analogSensors_arr[i]);
    }
    delay_ms(600);

    if (sensors == 0) {
      for (int i = 0; i < 6; i++) {
        servo_positions[vertical_leg_num[i]] = vertical_max[i];
      }
    } else if (sensors == 1) {
      for (int i = 0; i < 6; i++) {
        servo_positions[horizontal_leg_num[i]] = horizontal_max[i];
      }
    }
    update_all_registers_fast();
    delay_ms(600);

    for (int i = 0; i < 6; i++) {
      sensor_max[i] = analogRead(analogSensors_arr[i]);
    }
    delay_ms(600);
  }
  else{
    stand();
    delay_ms(5000);
    if (sensors == 0) {
      if (gait_num != -1) {
        for (int k = 0; k < 2; k++) {
          for (int i = 0; i < 6; i += 2) {
            servo_positions[vertical_leg_num[i]] = vertical_max[i];
            servo_positions[vertical_leg_num[i + 1]] = vertical_min[i + 1];
          }
          update_all_registers_fast();
          delay_ms(1100);
          for (int i = 0; i < 6; i += 2) {
            sensor_max[i] = analogRead(analogSensors_arr[i]);
            sensor_min[i + 1] = analogRead(analogSensors_arr[i + 1]);
          }
          delay_ms(1100);
          for (int i = 0; i < 6; i++) {
            servo_positions[vertical_leg_num[i]] = vertical_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          for (int i = 0; i < 6; i += 2) {
            servo_positions[vertical_leg_num[i + 1]] = vertical_max[i + 1];
            servo_positions[vertical_leg_num[i]] = vertical_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
          for (int i = 0; i < 6; i += 2) {
            sensor_max[i + 1] = analogRead(analogSensors_arr[i + 1]);
            sensor_min[i] = analogRead(analogSensors_arr[i]);
          }
          delay_ms(1100);
          for (int i = 0; i < 6; i++) {
            servo_positions[vertical_leg_num[i]] = vertical_min[i];
          }
          update_all_registers_fast();
          delay_ms(1100);
        }
      }
    }
  }

  stand();

  if (printSensorLimits) { // print all values
    serial->println("Calibration");
    serial->print("sensor\t");
    serial->print(sensors);
    serial->println(" min:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(sensor_min[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->println("\n");
    serial->print("sensor\t");
    serial->print(sensors);
    serial->println(" max:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(sensor_max[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->println("\n");
    serial->println("vertical min:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(vertical_min[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->println("\n");

    serial->println("vertical max:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(vertical_max[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->println("\n");

    serial->println("horizontal min:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(horizontal_min[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
    serial->println("\n");

    serial->println("horizontal max:\t");
    for (int i = 0; i < 6; i++) {
      serial->print(horizontal_max[i]);
      if (i < 5) {
        serial->print(", ");
      }
    }
  }
}

void Servotor32::sendToPC(int *data, Stream *serial) { // buffer selected data and write to serial stream
  byte buf[2 * inputCount]; // buffer size in bytes is determined by the number of written observables
  // with arduino integer input we have two bytes per integer value
  for (int i = 0; i < inputCount; i++) {
    buf[2 * i] = ((byte * )(data + i))[0]; // first byte split of integer value
    buf[2 * i + 1] = ((byte * )(data + i))[1]; // second byte split of integer value
  }
  serial->write(buf, 2 * inputCount); // write buffered data to serial
}

short Servotor32::tallyCount() {
  total = 0;
  for (int i = 0; i < numCount; i++) {
    total += powers[i] * numString[(numCount - 1) - i];
  }
  if (numCount == 0) {
    total = -1;
  }
  return total;
}

#define MAX_TIME 1000000

float Servotor32::ping() {
  //PB0 for Trigger (17)
  //PB7 for Echo (11)

  pinMode(17, OUTPUT);
  pinMode(11, INPUT);

  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);

  long duration;
  float cm;
  digitalWrite(17, LOW);
  delayMicroseconds(2);
  digitalWrite(17, HIGH);
  delayMicroseconds(10);
  digitalWrite(17, LOW);

  uint8_t bit = digitalPinToBitMask(11);
  uint8_t port = digitalPinToPort(11);
  uint8_t stateMask = (HIGH ? bit : 0);

  unsigned long startCount = 0;
  unsigned long endCount = 0;
  unsigned long width = 0; // keep initialization out of time critical area

  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = 500;

  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;

  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;

  startCount = micros_new();
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    delayMicroseconds(10); //loop 'jams' without this
    if ((micros_new() - startCount) > 58000) { // 58000 = 1000CM
      return 0;
      break;
    }
  }
  duration = micros_new() - startCount;
  //--------- end pulsein
  cm = (float) duration / 29.0 / 2.0;
  return cm;
}

float Servotor32::multiPing(unsigned short attempts = 5) {
  float distances[attempts];
  for (int i = 0; i < attempts; i++) {
    distances[i] = ping();
  }

  // sort them in order
  int i, j;
  float temp;

  for (i = (attempts - 1); i > 0; i--) {
    for (j = 1; j <= i; j++) {
      if (distances[j - 1] > distances[j]) {
        temp = distances[j - 1];
        distances[j - 1] = distances[j];
        distances[j] = temp;
      }
    }
  }

  // return the middle entry
  return distances[(int) ceil((float) attempts / 2.0)];

}