#include <Servotor32.h> // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

void setup() {
  hexy.begin();
}

void loop() {
  // blink the status led
  digitalWrite(STATUS_LED, HIGH);
  hexy.delay_ms(500); // wait 500mS
  digitalWrite(STATUS_LED, LOW);
  hexy.delay_ms(500); // wait 500mS
   
  while(true){
    hexy.process(&Serial1); //process input from the board serial (i.e. bluetooth)
    hexy.process(&Serial); //process input from the USB
  }
  
}
