/*
All hardware for Servotor32 is licensed under a Creative Commons Attribution ShareAlike 3.0 United States License.
The full terms of this license can be read at:
http://creativecommons.org/licenses/by-sa/3.0/

All software is licensed under the MIT License.
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Servotor32.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <math.h>

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

Servotor32::Servotor32()
{  

}
//stores information about the servos and groups
//signed short servo_positions[SERVOS]; // where the servos are currently (supposed to be) at
int servo_positions[SERVOS];
signed char servos_sorted[GROUPS][SERVOS_PER_GROUP]; // index in servo_timings to where the servo ends
signed char servos_active_in_group[GROUPS]; // the number of servos in a group currently active
uint8_t active_servos_hex[GROUPS];

// all updates to shift registers in order of their updates
signed short servo_timings[MAX_TIMINGS]; // the timing where the change occurs
uint8_t  shift_output[MAX_TIMINGS];  // the output of the shift register
uint8_t  shift_latch[MAX_TIMINGS];   // the shift register latch used

// keeps track of whether its safe or not to update the servos
uint8_t update_reg_flag = 0;

// variables for the callback
uint16_t timer;
uint8_t  counter = 0;
uint8_t  pwm_active = 1;

uint16_t group_offsets[4] = {0,251,502,753};
uint8_t group_latches[4] = {5,6,7,4};
uint8_t pin_2_num[8] = {0x08,0x04,0x02,0x01, 0x80,0x40,0x20,0x10};

void Servotor32::begin(){
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs
  
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

  for(byte i=0; i<SERVOS; i++){
    servo_positions[i] = -1;
  }
  for(byte i=0; i<GROUPS; i++){
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      servos_sorted[i][j] = -1;
    }
  }
  
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
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

long unsigned int Servotor32::micros_new(){
  return us_counter;
}

long unsigned int Servotor32::millis_new(){
  return us_counter/1000;
}

void Servotor32::delay_ms(long unsigned int delay_time){
  startTime = millis_new();
  currentTime = millis_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = millis_new() - startTime;
  }
}

void Servotor32::delay_us(long unsigned int delay_time){
  startTime = micros_new();
  currentTime = micros_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = micros_new() - startTime;
  }
}

void Servotor32::callback(){
  cli();
  if(timer < 1100){ // keep it from updating servos mid-array change by some weird coincidence
    if(timer == servo_timings[counter]){ // if the time has arrived to update a shift reg
      SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
      while(!(SPSR & (1<<SPIF))); //wait till the register completes
      PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
      PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
      counter++;
    }
  }

  timer++;
  us_counter += 10;
  if(timer == 1100){ // all servo timing completed
    update_reg_flag = 1; // allow updates to the timing arrays
  }
  if(timer == 1900){ // getting close to servo start-up again,
    update_reg_flag = 0; // don't allow any new timing array updates
  }
  if(timer == 2000){
    timer=0;
    counter=0;
  }
  sei();
}

void Servotor32::delete_from_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<servos_active_in_group[group]; i++){ // go through all the servos
    if(servos_sorted[group][i] == servo){ // find its place
       for(signed char j=i; j<servos_active_in_group[group]-1; j++){//move all servos in front of it back by one
         servos_sorted[group][j] = servos_sorted[group][j+1];
       }
       servos_sorted[group][servos_active_in_group[group]-1] = -1; //insert a -1 at the end of the move
       break; //break out of previous for loop, now that the job is done
    }
  }
  active_servos_hex[group] &= ~pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] -= 1;// decrease the number of active servos in the group by 1
}

void Servotor32::add_to_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<=servos_active_in_group[group]; i++){ // find the servo
     if(servos_sorted[group][i] == -1){ // if no servos yet entered, set as first
       servos_sorted[group][i] = servo; //insert the servo in its sorted place
       break; //stop the for loop, as the job is done
     }
     else{
       if(servo_positions[servos_sorted[group][i]] > pos){ // if this servo should go before this one
         for(signed char j=servos_active_in_group[group]-1; j>=i; j--){// move all others forward one
           servos_sorted[group][j+1] = servos_sorted[group][j];
         }
         servos_sorted[group][i] = servo; //insert the servo in its sorted place
         break;
       }
     }
  }
  active_servos_hex[group] |= pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] += 1;
}

void Servotor32::update_all_registers_fast(){
  while(update_reg_flag == 0){ // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- delete all ------
  for(byte group=0; group<GROUPS; group++){
    servos_active_in_group[group] = 0;
    active_servos_hex[group] = 0;
    for(byte i=0; i<SERVOS_PER_GROUP; i++){
      servos_sorted[group][i] = -1;
    }
  }
  
  // ----- add all active servos ------
  for(byte i=0; i<SERVOS; i++){
    if (servo_positions[i] != -1) {
      servos_sorted[i/SERVOS_PER_GROUP][servos_active_in_group[i/SERVOS_PER_GROUP]] = i; //index of active servo
      servos_active_in_group[i/SERVOS_PER_GROUP] += 1;
      active_servos_hex[i/SERVOS_PER_GROUP] |= pin_2_num[i%SERVOS_PER_GROUP];
    }
  }
  
  // ----- bubble sort servos_sorted ------
  // sort the array by servos position, the ones with pos = -1 will stay at the end
  boolean sorted = false;
  byte j = 0;
  short temp = 0;
  for(byte group=0; group<GROUPS; group++){   // for each group separately 
     sorted = false;
     j = 0;
     while (!sorted){   // continue sorting the array until is sorted. 
       sorted = true;  // expecting the the array sorted
       for (byte i=0; i<servos_active_in_group[group]-1-j; i++){   // go through the active servo list (they are at the front)
         if (servo_positions[servos_sorted[group][i]] > servo_positions[servos_sorted[group][i+1]]){  //if wrong order of two consecutive, swap them
           temp = servos_sorted[group][i];
           servos_sorted[group][i] = servos_sorted[group][i+1];
           servos_sorted[group][i+1] = temp;
           sorted = false;   // no, it was not sorted yet
         }
       }
       j++;
     }
  }

  // ----- create timing idicies from servo/group data ------- (no change from here on)
  
  // clear the timing arrays for fresh start
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index=0;
  uint8_t current_timing=0;
  uint8_t current_shift_output=0; 
  
  for(byte group=0; group<GROUPS; group++){ //go through each group
    if(servos_active_in_group[group] > 0){ // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1<<group_latches[group]);
      counter_index +=1;
      
      
      //create additional timings
      for(byte i=0; i<servos_active_in_group[group]; i++){ //create the timings for each servo after that, using the previous output
        if(servo_positions[servos_sorted[group][i]] == servo_positions[servos_sorted[group][i-1]]){ // if this servo's time is the same as the last's
          if(i != 0){
            counter_index -= 1; //reverse the index count-up
          }
          else{
            current_shift_output = shift_output[counter_index-1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
            shift_latch[counter_index] = (1<<group_latches[group]);
          }
        }
        else{
          current_shift_output = shift_output[counter_index-1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
          shift_latch[counter_index] = (1<<group_latches[group]);
        }
        
        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i]-group*SERVOS_PER_GROUP]; 
        shift_output[counter_index] = current_shift_output;
        counter_index +=1;
      }
    }      
  }
  
}

void Servotor32::update_registers_fast(byte servo, signed short pos){
  byte group = servo/8;
  while(update_reg_flag == 0){ // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- put the servo into, or take it out of its sorted array ------
  
  if(pos > 0){ // if the sevo isn't a kill command, then its an add/change
    if(servo_positions[servo] == -1){// if the servo is inactive
      // insert the servo into the array sorted
      add_to_sorted_array(servo,group,pos);
    }
    else{
      // updating the servo. First delete its existing entry, then insert it

      delete_from_sorted_array(servo,group,pos);
      add_to_sorted_array(servo,group,pos);
    }
  }
  else{ // servo is a kill command
    if(servo_positions[servo] != -1){ // make sure its even on first
      delete_from_sorted_array(servo,group,pos);
    }
  }
  
  servo_positions[servo] = pos;
  
  // ----- create timing idicies from servo/group data -------
  
  // clear the timing arrays for fresh start
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index=0;
  uint8_t current_timing=0;
  uint8_t current_shift_output=0; 
  
  for(byte group=0; group<GROUPS; group++){ //go through each group
    if(servos_active_in_group[group] > 0){ // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1<<group_latches[group]);
      counter_index +=1;
      
      
      //create additional timings
      for(byte i=0; i<servos_active_in_group[group]; i++){ //create the timings for each servo after that, using the previous output
        if(servo_positions[servos_sorted[group][i]] == servo_positions[servos_sorted[group][i-1]]){ // if this servo's time is the same as the last's
          if(i != 0){
            counter_index -= 1; //reverse the index count-up
          }
          else{
            current_shift_output = shift_output[counter_index-1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
            shift_latch[counter_index] = (1<<group_latches[group]);
          }
        }
        else{
          current_shift_output = shift_output[counter_index-1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
          shift_latch[counter_index] = (1<<group_latches[group]);
        }
        
        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i]-group*SERVOS_PER_GROUP]; 
        shift_output[counter_index] = current_shift_output;
        counter_index +=1;
      }
    }      
  }
  
}


void Servotor32::printStatus(Stream *serial){
  serial->println("--------------------- Registers ----------------------");
  
  serial->println("Servo Data:");
  serial->println("Servo\tPos\tTimeEnd\t");
  for(byte i=0; i<SERVOS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servo_positions[i]);
    serial->println("");
  }
  serial->println("");

  serial->println("Sorted Groups");
  for(byte i=0; i<GROUPS; i++){
    serial->print("Group: ");
    serial->println(i);
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      serial->print("Servo: ");
      serial->print(servos_sorted[i][j]);
      serial->print("\t");
      serial->println(servo_positions[servos_sorted[i][j]]);
      
    }
  }  

  serial->println("Group Data:");
  serial->println("#\tActive\tHex");
  for(byte i=0; i<GROUPS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servos_active_in_group[i]);
    serial->print("\t");
    serial->println(active_servos_hex[i],HEX);
  }
  serial->println("");
  
  serial->println("Timings:");
  serial->println("Pos\tTiming\tOutput\tLatch");
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    serial->print(i);
    serial->print(":\t");
    serial->print(servo_timings[i]);
    serial->print(",\t");
    serial->print(shift_output[i],HEX);
    serial->print(",\t");
    serial->println(shift_latch[i],HEX);
  }
  serial->println("----------------------------------------------------");
}

// modify the state of a servo
void Servotor32::changeServo(byte servo, short pos){
  if(pos == 0){
    pos = -1;
  }
  if(pos == -1){
    update_registers_fast(servo, pos);
  }
  else{
    update_registers_fast(servo, pos/10);
  }
}

boolean debug = false;
boolean testMode = false;
boolean servoCounting = false;
boolean posCounting = false;

boolean localCalc = false;

byte numString[6];
int powers[] = {1,10,100,1000};

byte numCount = 0;
unsigned short total = 0;
short inServo = -1;
short inPos = -1;
short skip = -1;

//uint8_t servosInPacket[] = {5,6,7, 9,10,11, 13,14,15, 18,17,16, 22,21,20, 26,25,24, 31};
uint8_t servosInPacket[] = {6,7, 10,11, 14,15, 17,16, 21,20, 25,24, 5,9,13,18,22,26};  // rearranged leg numbering

float dt = 0.1f;
long unsigned int time_null = 0;
long unsigned int step_count = 0;

//sensor configuration
boolean sensor_horizontal = false;  //only sensors of horizontal motors
boolean sensor_horizontal_vertical = false;  //all horizontal sensors and two vertical sensors
boolean sensor_vertical = false;  //only sensors of vertical motors
boolean open_loop_horiz = false;
boolean open_loop_vert = true;  //no sensor feedback

int direction_left = 1;  //+1 for forward direction; -1 backwards
int direction_right = 1;

boolean onspot = false;
boolean stomp = true;

//gait
float tripod = 0;
float tetrapod = 0;
float wave = 1;

//parameters
//equal for all gaits:
float gain_vert = 6.0f;
float tau_b = 0.15f;
float tau_x = 0.15f;

//tripod gait
float neuron_avg_tri = 0.4f; //20% decreased from 1/2
float weight_tri = 1.5f;

//tetrapod gait
float neuron_avg_quad = 0.27f; //20% decreased from 1/3
float weight_quad = 2.5f;

//wave
float neuron_avg_wave = 0.13f; //20% decreased from 1/6
float weight_wave = 4.5f;

//horizontal motion range and corresponding sensor range
uint8_t horizontal_leg_num[6] = {7, 11, 15, 16, 20, 24};

float horizontal_min[6] = {185, 175, 135, 155, 135, 95};
float sensor_horiz_min[6] = {402, 366, 251, 318, 370, 174}; //sensor values of motor #16 and #20 (replacement motor) are reversed

float horizontal_max[6] = {145, 135, 95, 195, 175, 135};
float sensor_horiz_max[6] = {327, 291, 175, 213, 274, 249};

//vertical motion range and corresponding sensor range
uint8_t vertical_leg_num[6] = {6, 10, 14, 17, 21, 25};

float vertical_min[6] = {190, 210, 205, 200, 215, 205};
//float sensor_vert_min[6] = {238.0, 196.0, 187.0, 197.0, 183.0, 173.0};  //tripod cal.
//float sensor_vert_min[6] = {242.0, 194.0, 178.0, 186.0, 181.0, 177.0};  //tetrapod cal.
//float sensor_vert_min[6] = {237.0, 177.0, 177.0, 184.0, 177.0, 172.0}; //wave cal.
float sensor_vert_min[6] = {235.0, 173.0, 170.0, 182.0, 141.0, 170.0}; //air cal.

float vertical_max[6] = {150, 170, 165, 160, 170, 165};
//float vertical_max[6] = {140, 160, 155, 150, 145, 155}; //horizontal alignment
float sensor_vert_max[6] = {355.0, 289.0, 292.0, 302.0, 277.0, 288.0};


uint8_t tibia_leg_num[6] = {5, 9, 13, 18, 22, 26};

float tibia_pos[6] = {110, 115, 130, 120, 125, 110};  //tibia stays in the same position

//uint8_t tibia_max[6] = {140, 150, 160, 150, 160, 140}; // horizontal alignment; 180 degrees to vertical max
//uint8_t tibia_min[6] = {80, 80, 90, 80, 90, 75}; //90 degrees to vertical max

//initial conditions
//tripod
//float shift_vert[6] = {2.41, 0.16, 2.86, 0.20, 3.19, 0.14};
//float shift_vert_old[6] = {2.41, 0.16, 2.86, 0.20, 3.19, 0.14};
//float shift_vert_diff_old[6] = {0.40, -0.27, 0.39, -0.27, 0.39, -0.27};
//float shift_vert_min[6] = {-0.59, -0.53, -0.31, -0.00, -0.11, -0.19};
//float shift_vert_max[6] = {2.79, 2.82, 3.07, 3.09, 3.27, 3.07};
//float shift_horiz[6] = {2.40, 0.23, 2.73, 0.28, 2.92, 0.06};
//float membrane[6] = {3.00, -3.00, 3.00, -3.00, 3.00, -3.00};
//float neuron_vert[6] = {1.00, 0.00, 1.00, 0.00, 1.00, 0.00};

//tetrapod
//float shift_vert[6] = {1.97, -0.08, 2.27, 2.09, 1.85, -0.27};
//float shift_vert_old[6] = {1.97, -0.08, 2.27, 2.09, 1.85, -0.27};
//float shift_vert_diff_old[6] = {0.55, -0.22, -0.22, 0.55, -0.22, -0.22};
//float shift_vert_min[6] = {-1.75, -1.39, -1.57, -1.64, -1.41, -1.65};
//float shift_vert_max[6] = {3.80, 4.26, 4.02, 3.97, 3.88, 4.20};
//float shift_horiz[6] = {1.12, -0.42, 3.86, 1.13, 3.05, -0.77};
//float membrane[6] = {4.00, -4.00, 0.00, 4.00, 0.00, -4.00};
//float neuron_vert[6] = {1.00, 0.00, 0.00, 1.00, 0.00, 0.00};

//wave
float shift_vert[6] = {-1.64, 0.02, 1.05, 3.04, 4.19, 4.70};
float shift_vert_old[6] = {-1.64, 0.02, 1.05, 3.04, 4.19, 4.70};
float shift_vert_diff_old[6] = {-0.16, -0.16, -0.16, -0.16, -0.16, 0.83};
float shift_vert_min[6] = {-1.72, -1.77, -1.83, -1.89, -1.46, -1.79};
float shift_vert_max[6] = {4.92, 5.29, 4.87, 5.57, 5.31, 5.07};
float shift_horiz[6] = {-0.94, -0.42, 1.71, 4.08, 4.72, 4.22};
float membrane[6] = {-5.00, -3.00, -1.00, 1.00, 3.00, 5.00};
float neuron_vert[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 1.00};

//variables
float feedback;
float shift_vert_avg = 0.0f;
float gain_horiz = 0.0f;

float neuron_horiz[6];

float neuron_avg;
float weight;

float analog_sensor[8];  //analog sensor array
float analog_sensor_norm[8];

float matrix[6][6];

//input variables for streaming buffered data
int input_zero;
int input_one;
int input_two;
int input_three;
int input_four;
int input_five;
int input_six;
int input_seven;
int input_eight;
int input_nine;
int input_ten;
int input_eleven;
int input_twelve;

float rangeshift_left = 0.0;
float rangeshift_right = 0.0;
float rangeshift_vert_min = 0.5;
float rangeshift_vert_max = 0.5;

//gait matrices:
int8_t matrix_tripod[6][6] = {
  {1, -1, 1, -1, 1, -1},
  {-1, 1, -1, 1, -1, 1},
  {1, -1, 1, -1, 1, -1},
  {-1, 1, -1, 1, -1, 1},
  {1, -1, 1, -1, 1, -1},
  {-1, 1, -1, 1, -1, 1}
};

int8_t matrix_tetrapod[6][6] = {
  {1, 0, -1, 1, -1, 0}, 
  {-1, 1, 0, -1, 0, 1}, 
  {0, -1, 1, 0, 1, -1}, 
  {1, 0, -1, 1, -1, 0}, 
  {0, -1, 1, 0, 1, -1}, 
  {-1, 1, 0, -1, 0, 1}
};

int8_t matrix_wave[6][6] = {
  {10, 6, 2, -2, -6, -10}, 
  {-10, 10, 6, 2, -2, -6}, 
  {-6, -10, 10, 6, 2, -2}, 
  {-2, -6, -10, 10, 6, 2},
  {2, -2, -6, -10, 10, 6}, 
  {6, 2, -2, -6, -10, 10}
};

//int8_t identity[6][6] = {
//  {1, 0, 0, 0, 0, 0}, 
//  {0, 1, 0, 0, 0, 0}, 
//  {0, 0, 1, 0, 0, 0}, 
//  {0, 0, 0, 1, 0, 0}, 
//  {0, 0, 0, 0, 1, 0}, 
//  {0, 0, 0, 0, 0, 1}
//};

void Servotor32::process(Stream *serial){
  if(serial->available()) { //process input from the USB
    char inChar = (char)serial->read();
    if (localCalc){
      switch(inChar){
        case '\r':
        case '\n':
          numCount = 0;
          localCalc = false;
          
          for (int i; i<6; i++) {
//            tibia_pos[i] -= 5;  
//            
//            vertical_min[i] = vertical_min[i] - rangeshift_vert_min * abs(vertical_max[i] - vertical_min[i]);
//            vertical_max[i] = vertical_max[i] + rangeshift_vert_max * abs(horizontal_max[i] - horizontal_min[i]);
//            
//            sensor_vert_min[i] = sensor_vert_min[i] - rangeshift_vert_min * abs(sensor_vert_max[i] - sensor_vert_min[i]);
//            sensor_vert_max[i] = sensor_vert_max[i] + rangeshift_vert_max * abs(sensor_vert_max[i] - sensor_vert_min[i]);
//            
            if ((i==3)||(i==4)||(i==5)) {
              sensor_horiz_min[i] = sensor_horiz_min[i] + rangeshift_right * abs(sensor_horiz_max[i] - sensor_horiz_min[i]);
              sensor_horiz_max[i] = sensor_horiz_max[i] + rangeshift_right * abs(sensor_horiz_max[i] - sensor_horiz_min[i]);
            }
            else {
              sensor_horiz_min[i] = sensor_horiz_min[i] - rangeshift_left * abs(sensor_horiz_max[i] - sensor_horiz_min[i]);
              sensor_horiz_max[i] = sensor_horiz_max[i] - rangeshift_left * abs(sensor_horiz_max[i] - sensor_horiz_min[i]);
            }
          }
          
          //superposition of gait matrices
          for (int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
              matrix[i][j] = tripod * matrix_tripod[i][j] + tetrapod * matrix_tetrapod[i][j] + wave/10. * matrix_wave[i][j];
//              matrix[i][j] = tripod * identity[i][j];
//              matrix[i][j] = tetrapod * identity[i][j];
//              matrix[i][j] = wave * identity[i][j];
            }
          }
                            
          //superposition of gait parameters
          neuron_avg = tripod * neuron_avg_tri + tetrapod * neuron_avg_quad + wave * neuron_avg_wave;
          weight = tripod * weight_tri + tetrapod * weight_quad + wave * weight_wave;
          
          for(int i=0; i<6; i++){
            changeServo(horizontal_leg_num[i], (horizontal_max[i] + horizontal_min[i])/2*10);
            changeServo(vertical_leg_num[i], vertical_min[i]*10);
            changeServo(tibia_leg_num[i], tibia_pos[i]*10);
          }
          
          delay(1100);

          while(inChar != 'K') {  //calculation runs until 'K' is received via serial port
            time_null = micros_new();  //initialization to measure time step

            inChar = (char)serial->read();  //read serial input every step
            
            if (serial->available() != 0) {  // change parameter on the run
//              serial->println("Input read!");
              switch(inChar) {
                case 'G':
                  weight += 0.25;
                  break;
                case 'g':
                  weight -= 0.25;
                  break;
                case 'A':
                  gain_vert += 0.25;
                  break;
                case 'a':
                  gain_vert -= 0.25;
                  break;
                case 'H':
                  onspot = true;
                  break;
                case 'h':
                  onspot = false;
                  break;
                case 'V':
                  stomp = true;
                  break;
                case 'v':
                  stomp = false;
                  break;
                case 's':
                  tau_b -= 0.05;
                  tau_x -= 0.05;
                  serial->println("tau: " + String(tau_b));
                  break;
                case 'S':
                  tau_b += 0.05;
                  tau_x += 0.05;
                  serial->println("tau: " + String(tau_b));
                  break;
                case 'd':
                  direction_left *= -1;
                  direction_right *= -1;
                  break;                
                case 'D':
                  direction_left *= -1;
                  break;
                case 'R':
                  rangeshift_right += 0.1;
                  break;
                case 'r':
                  rangeshift_right -= 0.1;
                  break;
                case 'L':
                  rangeshift_left += 0.1;
                  break;
                case 'l':
                  rangeshift_left -= 0.1;
                  break;
                case 't':
                  neuron_avg = neuron_avg_tri;
                  break;
                case 'q':
                  neuron_avg = neuron_avg_quad;
                  break;
                case 'w':
                  neuron_avg = neuron_avg_wave;
                  break;
                case 'T':
                  for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                      matrix[i][j] = matrix_tripod[i][j];
                    }
                  }
                  weight = weight_tri;
                  break;
                case 'Q':
                  for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                      matrix[i][j] = matrix_tetrapod[i][j];
                    }
                  }
                  weight = weight_quad;
                  break;
                case 'W':
                  for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                      matrix[i][j] = matrix_wave[i][j];
                    }
                  }
                  weight = weight_wave;
                  break;
              }
            }
                        
            //analogRead() reads out the pins A4-A11 where the encoders of the motors are plugged in
            analog_sensor[0] = analogRead(A4);
            analog_sensor[1] = analogRead(A5);
            analog_sensor[2] = analogRead(A6);
            analog_sensor[3] = analogRead(A7);
            analog_sensor[4] = analogRead(A8);
            analog_sensor[5] = analogRead(A9);
            
//            if (sensor_horizontal_vertical) {
//              analog_sensor[6] = analogRead(A10);
//              analog_sensor[7] = analogRead(A11);
//              analog_sensor_norm[6] = (analog_sensor[6] - sensor_vert_min[0])/(sensor_vert_max[0] - sensor_vert_min[0]);
//              analog_sensor_norm[7] = (analog_sensor[7] - sensor_vert_min[5])/(sensor_vert_max[5] - sensor_vert_min[5]);
//            }
            
            for (int i=0; i<6; i++) {
              if (sensor_horizontal || sensor_horizontal_vertical || open_loop_horiz) {
                analog_sensor_norm[i] = (analog_sensor[i] - sensor_horiz_min[i])/(sensor_horiz_max[i] - sensor_horiz_min[i]);
              }
              else if (sensor_vertical || open_loop_vert) {
                if  (wave == 1) {
                  if (step_count < 100) {
                    analog_sensor_norm[i] = neuron_vert[i];
                  }
                  else {
                    analog_sensor_norm[i] = 1.0*(analog_sensor[i] - sensor_vert_min[i])/(sensor_vert_max[i] - sensor_vert_min[i]) + 0.0 * neuron_vert[i];
                    if (analog_sensor_norm[i] > 0.5) {
                      analog_sensor_norm[i] = 1;
                    }
                    if (analog_sensor_norm[i] < 0.5) {
                      analog_sensor_norm[i] = 0;
                    }
                  }
                }
                else {
                  analog_sensor_norm[i] = (analog_sensor[i] - sensor_vert_min[i])/(sensor_vert_max[i] - sensor_vert_min[i]);
                }
              }
            }
                                    
            for (int i=0; i<6; i++){
              feedback = 0.;  //reset feedback input
              
              if (sensor_horizontal) {
                for (int j=0; j<6; j++){
                  feedback += matrix[i][j] * neuron_vert[j];
                }
                if ((i==3)||(i==4)||(i==5)) {
                  shift_horiz[i] = shift_vert_avg + direction_right * ((1-analog_sensor_norm[i]) * (shift_vert_min[i] - shift_vert_avg) + analog_sensor_norm[i] * (shift_vert_max[i] - shift_vert_avg));
                }
                else {
                  shift_horiz[i] = shift_vert_avg + direction_left * ((1-analog_sensor_norm[i]) * (shift_vert_min[i] - shift_vert_avg) + analog_sensor_norm[i] * (shift_vert_max[i] - shift_vert_avg));
                }
                
                membrane[i] += 1./tau_x * (weight * feedback - membrane[i]) * dt;
                neuron_vert[i] = 1/(1+exp(gain_vert * (shift_horiz[i] - membrane[i])));
              }
              
              else if (sensor_vertical) {
                for(int j=0; j<6; j++){
                  feedback += matrix[i][j] * analog_sensor_norm[j];
                }
                membrane[i] += 1./tau_x * (weight * feedback - membrane[i]) * dt;
                neuron_vert[i] = 1/(1+exp(gain_vert * (shift_vert[i] - membrane[i])));
              }
              
//              else if (sensor_horizontal_vertical) {
//                for(int j=0; j<6; j++){
//                  if (j==0){
//                    feedback += matrix[i][j] * analog_sensor_norm[6];  //additional vertical sensor reading on leg0
//                  }
//                  else if (j==5) {
//                    feedback += matrix[i][j] * analog_sensor_norm[7];  //additional vertical sensor reading on leg5
//                  }
//                  else {
//                    feedback += matrix[i][j] * neuron_vert[j];  //remaining legs do not have vertical sensor reading
//                  }
//                }
//                shift_horiz[i] = shift_vert_avg + direction * ((1-analog_sensor_norm[i]) * (shift_vert_min[i] - shift_vert_avg) + analog_sensor_norm[i] * (shift_vert_max[i] - shift_vert_avg));
//                membrane[i] += 1./tau_x * (weight * feedback - membrane[i]) * dt;
//                neuron_vert[i] = 1/(1+exp(gain_vert * (shift_horiz[i] - membrane[i])));
//              }
              
              else if (open_loop_horiz || open_loop_vert) {
                for(int j=0; j<6; j++){
                  feedback += matrix[i][j] * neuron_vert[j];
                }
                membrane[i] += 1./tau_x * (weight * feedback - membrane[i]) * dt;
                neuron_vert[i] = 1/(1+exp(gain_vert * (shift_vert[i] - membrane[i])));
              }
              
              shift_vert[i] += 1./tau_b * (neuron_vert[i] - neuron_avg) * dt;
                            
              //determine extrema of vertical shifts (b_min, b_max)
              if (((shift_vert[i] - shift_vert_old[i]) < 0) && ((shift_vert[i] - shift_vert_old[i]) * shift_vert_diff_old[i] < 0)) {
                shift_vert_max[i] = shift_vert_old[i];
              } 
              if (shift_vert[i] > shift_vert_max[i]) {
                shift_vert_max[i] = shift_vert[i];
              }
              if (((shift_vert[i] - shift_vert_old[i]) > 0) && ((shift_vert[i] - shift_vert_old[i]) * shift_vert_diff_old[i] < 0)) {
                shift_vert_min[i] = shift_vert_old[i];
              }
              if (shift_vert[i] < shift_vert_min[i]) {
                shift_vert_min[i] = shift_vert[i];
              }
              
              //store data to use as previous step
              shift_vert_diff_old[i] = shift_vert[i] - shift_vert_old[i];
              shift_vert_old[i] = shift_vert[i];
              
              shift_vert_avg = (shift_vert_max[i] + shift_vert_min[i])/2;
              
              if ((i==3)||(i==4)||(i==5)) {
                gain_horiz = direction_right * 2 * log(1/0.98-1)/(shift_vert_min[i] - shift_vert_max[i]);
              }
              else {
                gain_horiz = direction_left * 2 * log(1/0.98-1)/(shift_vert_min[i] - shift_vert_max[i]);
              }
              
              neuron_horiz[i] = 1/(1+exp(gain_horiz * (shift_vert_avg - shift_vert[i])));
              
              
              //  storing next motor positions in array
              if (stomp == true) {
                servo_positions[vertical_leg_num[i]] = (1-neuron_vert[i]) * vertical_min[i] + neuron_vert[i] * vertical_max[i];
              }
              else {
                servo_positions[vertical_leg_num[i]] = vertical_min[i];
              }
              
              if (onspot == false) {
                if ((i==3)||(i==4)||(i==5)) {
                  servo_positions[horizontal_leg_num[i]] = (1-neuron_horiz[i]) * (horizontal_min[i] + rangeshift_right * abs(horizontal_max[i]-horizontal_min[i])) + neuron_horiz[i] * (horizontal_max[i] + rangeshift_right * abs(horizontal_max[i]-horizontal_min[i]));
                }
                else{
                  servo_positions[horizontal_leg_num[i]] = (1-neuron_horiz[i]) * (horizontal_min[i] - rangeshift_left * abs(horizontal_max[i]-horizontal_min[i])) + neuron_horiz[i] * (horizontal_max[i] - rangeshift_left * abs(horizontal_max[i]-horizontal_min[i]));
                }
              }
              else {
                servo_positions[horizontal_leg_num[i]] = (horizontal_max[i] + horizontal_min[i])/2;
              }
            }
            
            update_all_registers_fast();  // update motor positions
            
            //int input variables for serial streaming
            input_zero = dt * 1000;
            
            input_one  = neuron_vert[0] * 1000;
            input_two = neuron_vert[1] * 1000;
            input_three = neuron_vert[2] * 1000;
            input_four = neuron_vert[3] * 1000;
            input_five = neuron_vert[4] * 1000;
            input_six = neuron_vert[5] * 1000;
            input_seven  = analog_sensor_norm[0] * 1000;
            input_eight = analog_sensor_norm[1] * 1000;
            input_nine = analog_sensor_norm[2] * 1000;
            input_ten = analog_sensor_norm[3] * 1000;
            input_eleven = analog_sensor_norm[4] * 1000;
            input_twelve = analog_sensor_norm[5] * 1000; 
            
            //sending buffered int data via serial
            sendToPC(&input_zero, &input_one, &input_two, &input_three, &input_four, &input_five, &input_six, &input_seven, &input_eight, &input_nine, &input_ten, &input_eleven, &input_twelve, serial);
            
            step_count++;
            dt = (micros_new() - time_null) * 0.000001;
            
            //further streaming combinations
//            input_one = analog_sensor[0] * 10;
//            input_two = analog_sensor[1] * 10;
//            input_three = analog_sensor[2] * 10;
//            input_four = analog_sensor[3] * 10;
//            input_five = analog_sensor[4] * 10;
//            input_six = analog_sensor[5] * 10;
//            input_seven  = analog_sensor_norm[0] * 1000;
//            input_eight = analog_sensor_norm[1] * 1000;
//            input_nine = analog_sensor_norm[2] * 1000;
//            input_ten = analog_sensor_norm[3] * 1000;
//            input_eleven = analog_sensor_norm[4] * 1000;
//            input_twelve = analog_sensor_norm[5] * 1000;
            
//            input_one = shift_vert[0] * 1000;
//            input_two = shift_vert[1] * 1000;
//            input_three = shift_vert[2] * 1000;
//            input_four = neuron_vert[0] * 1000;
//            input_five = neuron_vert[1] * 1000;
//            input_six = neuron_vert[2] * 1000;
//            input_seven  = membrane[0] * 1000;
//            input_eight = membrane[1] * 1000;
//            input_nine = membrane[2] * 1000;
//            input_ten = analog_sensor_norm[0] * 1000;
//            input_eleven = analog_sensor_norm[1] * 1000;
//            input_twelve = analog_sensor_norm[2] * 1000;
            
//            input_one = shift_vert[0] * 1000;
//            input_two = shift_vert[1] * 1000;
//            input_three = shift_vert[2] * 1000;
//            input_four = neuron_vert[0] * 1000;
//            input_five = neuron_vert[1] * 1000;
//            input_six = neuron_vert[2] * 1000;
//            input_seven  = neuron_horiz[0] * 1000;
//            input_eight = neuron_horiz[1] * 1000;
//            input_nine = neuron_horiz[2] * 1000;
//            input_ten = analog_sensor_norm[0] * 1000;
//            input_eleven = analog_sensor_norm[1] * 1000;
//            input_twelve = analog_sensor_norm[2] * 1000;
            
//            input_one = shift_vert[0] * 1000;
//            input_two = shift_vert[1] * 1000;
//            input_three = shift_vert[2] * 1000;
//            input_four = shift_vert[3] * 1000;
//            input_five = shift_vert[4] * 1000;
//            input_six = shift_vert[5] * 1000;
//            input_seven  = neuron_vert[0] * 1000;
//            input_eight = neuron_vert[1] * 1000;
//            input_nine = neuron_vert[2] * 1000;
//            input_ten = neuron_vert[3] * 1000;
//            input_eleven = neuron_vert[4] * 1000;
//            input_twelve = neuron_vert[5] * 1000;
            
//            input_one = neuron_horiz[0] * 1000;
//            input_two = neuron_horiz[1] * 1000;
//            input_three = neuron_horiz[2] * 1000;
//            input_four = neuron_horiz[3] * 1000;
//            input_five = neuron_horiz[4] * 1000;
//            input_six = neuron_horiz[5] * 1000;
//            input_seven  = analog_sensor_norm[0] * 1000;
//            input_eight = analog_sensor_norm[1] * 1000;
//            input_nine = analog_sensor_norm[2] * 1000;
//            input_ten = analog_sensor_norm[3] * 1000;
//            input_eleven = analog_sensor_norm[4] * 1000;
//            input_twelve = analog_sensor_norm[5] * 1000;

//            input_one = shift_vert[0] * 1000;
//            input_two = shift_vert[1] * 1000;
//            input_three = shift_vert[2] * 1000;
//            input_four = shift_vert[3] * 1000;
//            input_five = shift_vert[4] * 1000;
//            input_six = shift_vert[5] * 1000;
//            input_seven  = analog_sensor_norm[0] * 1000;
//            input_eight = analog_sensor_norm[1] * 1000;
//            input_nine = analog_sensor_norm[2] * 1000;
//            input_ten = analog_sensor_norm[3] * 1000;
//            input_eleven = analog_sensor_norm[4] * 1000;
//            input_twelve = analog_sensor_norm[5] * 1000;
            
//            input_one = neuron_horiz[0] * 1000;
//            input_two = neuron_horiz[1] * 1000;
//            input_three = neuron_horiz[2] * 1000;
//            input_four = neuron_horiz[3] * 1000;
//            input_five = neuron_horiz[4] * 1000;
//            input_six = neuron_horiz[5] * 1000;
//            input_seven  = neuron_vert[0] * 1000;
//            input_eight = neuron_vert[1] * 1000;
//            input_nine = neuron_vert[2] * 1000;
//            input_ten = neuron_vert[3] * 1000;
//            input_eleven = neuron_vert[4] * 1000;
//            input_twelve = neuron_vert[5] * 1000;
            
//            input_one = shift_vert[0] * 1000;
//            input_two = shift_vert[1] * 1000;
//            input_three = shift_vert[2] * 1000;
//            input_four = shift_vert[3] * 1000;
//            input_five = shift_vert[4] * 1000;
//            input_six = shift_vert[5] * 1000;
//            input_seven  = neuron_vert[0] * 1000;
//            input_eight = neuron_vert[1] * 1000;
//            input_nine = neuron_vert[2] * 1000;
//            input_ten = neuron_vert[3] * 1000;
//            input_eleven = neuron_vert[4] * 1000;
//            input_twelve = neuron_vert[5] * 1000;
            
//            input_one = analog_sensor[0] * 10;
//            input_two = analog_sensor[1] * 10;
//            input_three = analog_sensor[2] * 10;
//            input_four = analog_sensor[3] * 10;
//            input_five = analog_sensor[4] * 10;
//            input_six = analog_sensor[5] * 10;            
//            input_seven  = neuron_vert[0] * 1000;
//            input_eight = neuron_vert[1] * 1000;
//            input_nine = neuron_vert[2] * 1000;
//            input_ten = neuron_vert[3] * 1000;
//            input_eleven = neuron_vert[4] * 1000;
//            input_twelve = neuron_vert[5] * 1000;            
          
            
            //print initial conditions
//            if (step_count == 300){
//              serial->println("b_h: {" + String(shift_horiz[0]) + ", " + String(shift_horiz[1]) + ", " + String(shift_horiz[2]) + ", " + String(shift_horiz[3]) + ", " + String(shift_horiz[4]) + ", " + String(shift_horiz[5]) + "}\n");
//              serial->println("y: {" + String(neuron_vert[0]) + ", " + String(neuron_vert[1]) + ", " + String(neuron_vert[2]) + ", " + String(neuron_vert[3]) + ", " + String(neuron_vert[4]) + ", " + String(neuron_vert[5]) + "}\n");
//              serial->println("x: {" + String(membrane[0]) + ", " + String(membrane[1]) + ", " + String(membrane[2]) + ", " + String(membrane[3]) + ", " + String(membrane[4]) + ", " + String(membrane[5]) + "}\n");
//              serial->println("b_max: {" + String(shift_vert_max[0]) + ", " + String(shift_vert_max[1]) + ", " + String(shift_vert_max[2]) + ", " + String(shift_vert_max[3]) + ", " + String(shift_vert_max[4]) + ", " + String(shift_vert_max[5]) + "}\n");
//              serial->println("b_min: {" + String(shift_vert_min[0]) + ", " + String(shift_vert_min[1]) + ", " + String(shift_vert_min[2]) + ", " + String(shift_vert_min[3]) + ", " + String(shift_vert_min[4]) + ", " + String(shift_vert_min[5]) + "}\n");
//              serial->println("b: {" + String(shift_vert[0]) + ", " + String(shift_vert[1]) + ", " + String(shift_vert[2]) + ", " + String(shift_vert[3]) + ", " + String(shift_vert[4]) + ", " + String(shift_vert[5]) + "}\n");
//              serial->println("b_old: {" + String(shift_vert_old[0]) + ", " + String(shift_vert_old[1]) + ", " + String(shift_vert_old[2]) + ", " + String(shift_vert_old[3]) + ", " + String(shift_vert_old[4]) + ", " + String(shift_vert_old[5]) + "}\n");
//              serial->println("b_diff_old: {" + String(shift_vert_diff_old[0]) + ", " + String(shift_vert_diff_old[1]) + ", " + String(shift_vert_diff_old[2]) + ", " + String(shift_vert_diff_old[3]) + ", " + String(shift_vert_diff_old[4]) + ", " + String(shift_vert_diff_old[5]) + "}\n");
//            }            
          }
          break;
      }
    }
    else {
      switch(inChar){
        case '&':  //starts local calculation
          localCalc = true;
          numCount = 0;
          break;
        case '#':
          servoCounting = true;
          numCount = 0;
          inServo = -1;
          inPos = -1;
          break;
        case 'D':
          printStatus(serial);
          break; 
        case 'P':
          if(servoCounting){
            inServo = tallyCount();
            servoCounting = false;
          }
          posCounting =  true;
          numCount = 0;
          break; 
        case '\r':
        case '\n':
          if(posCounting){
            inPos = tallyCount();
            posCounting = false;
          }
          if((inServo >=0)&&(inServo <=31)&&(((inPos >= 500)&&(inPos <= 2500))||(inPos == -1))){
            changeServo(inServo,inPos);  
            serial->print('#' + inServo + 'P' + inPos);    
            inServo = -1;
            inPos = -1;
          }
          numCount = 0;
          break;
        case 'V':
          serial->println("SERVOTOR32_v2.0mod");
          break;
        case 'C':
          for(int i=0; i<32; i++){
            changeServo(i, 1500);
          }
          serial->println("All Centered");
          break;
        case 'S': //walking pose
          for(int i=0; i<6; i++){
            changeServo(horizontal_leg_num[i], (horizontal_max[i] + horizontal_min[i])/2*10);
            changeServo(vertical_leg_num[i], vertical_min[i]*10);
            changeServo(tibia_leg_num[i], tibia_pos[i]*10);
          }
          serial->println("Ready to run");
          break;
        case 'T': //pose for transport
          for(int i=0; i<6; i++){
            changeServo(horizontal_leg_num[i], (horizontal_max[i] + horizontal_min[i])/2*10);
            changeServo(vertical_leg_num[i], 800);
            changeServo(tibia_leg_num[i], 800);
          }
          delay(5000);
          changeServo(horizontal_leg_num[0], 600);
          changeServo(horizontal_leg_num[5], 2400);
          changeServo(horizontal_leg_num[1], 800);
          changeServo(horizontal_leg_num[4], 2200);
          changeServo(horizontal_leg_num[2], 900);
          changeServo(horizontal_leg_num[3], 2000);
          serial->println("Ready to pack");
          break;
//        case 'N':
//          for(int i=0; i<6; i++){
//            changeServo(horizontal_leg_num[i], horizontal_min[i]*10);
//          }
//          serial->println("Horizontal range minimum");
//          break;
//        case 'X':
//          for(int i=0; i<6; i++){
//            changeServo(horizontal_leg_num[i], horizontal_max[i]*10);
//          }
//          serial->println("Horizontal range maximum");
//          break;
//        case 'M':
//          for(int i=0; i<6; i++){
//            changeServo(vertical_leg_num[i], vertical_min[i]*10);
//          }
//          serial->println("Vertical range minimum");
//          break;
//        case 'Y':
//          for(int i=0; i<6; i++){
//            changeServo(vertical_leg_num[i], vertical_max[i]*10);
//          }
//          serial->println("Vertical range maximum");
//          break;
        case 'K':
          for(int i=0; i<32; i++){
            changeServo(i,-1);
          }
          serial->println("All Turned Off");
          break;
        case 'L':
          if(servoCounting){
            inServo = tallyCount();
            servoCounting = false;
          }
          changeServo(inServo, -1);
          break;
        default:
          if((inChar > 47)&&(inChar < 58)){
            if(numCount<4){
              numString[numCount] = inChar-48;
              numCount++;
            }
          }
          break;
      }
    }
  }
}

short Servotor32::tallyCount(){
   total=0;
   for(int i=0; i<numCount; i++){  
     total += powers[i]*numString[(numCount-1)-i];  
   }
   if(numCount == 0){
     total = -1;
   }
   return total;
}

#define MAX_TIME 1000000

float Servotor32::ping(){
  //PB0 for Trigger (17)
  //PB7 for Echo (11)
  
  pinMode(17,OUTPUT);
  pinMode(11,INPUT);
  
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
    if((micros_new() - startCount) > 58000 ){ // 58000 = 1000CM
      return 0;
      break;
    }
  }
  duration = micros_new() - startCount;
  //--------- end pulsein
  cm = (float)duration / 29.0 / 2.0; 
  return cm;
}

float Servotor32::multiPing(unsigned short attempts=5){
  float distances [attempts];
  for(int i=0; i<attempts; i++){
    distances[i] = ping();
  }
  
  // sort them in order
  int i, j;
  float temp;
 
  for (i = (attempts - 1); i > 0; i--)
  {
    for (j = 1; j <= i; j++)
    {
      if (distances[j-1] > distances[j])
      {
        temp = distances[j-1];
        distances[j-1] = distances[j];
        distances[j] = temp;
      }
    }
  }
  
  // return the middle entry
  return distances[(int)ceil((float)attempts/2.0)];
  
}

void Servotor32::sendToPC(int* data0, int* data1, int* data2, int* data3, int* data4, int* data5, int* data6, int* data7, int* data8, int* data9, int* data10, int* data11, int* data12, Stream* serial)
{
  byte* byteData0 = (byte*)(data0);
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte* byteData7 = (byte*)(data7);
  byte* byteData8 = (byte*)(data8);
  byte* byteData9 = (byte*)(data9);
  byte* byteData10 = (byte*)(data10);
  byte* byteData11 = (byte*)(data11);
  byte* byteData12 = (byte*)(data12);
  byte buf[26] = {byteData0[0], byteData0[1],
                  byteData1[0], byteData1[1],
                  byteData2[0], byteData2[1],
                  byteData3[0], byteData3[1],
                  byteData4[0], byteData4[1],
                  byteData5[0], byteData5[1],
                  byteData6[0], byteData6[1],
                  byteData7[0], byteData7[1],
                  byteData8[0], byteData8[1],
                  byteData9[0], byteData9[1],
                  byteData10[0], byteData10[1],
                  byteData11[0], byteData11[1],
                  byteData12[0], byteData12[1]
                  };
  serial->write(buf, 26);
}