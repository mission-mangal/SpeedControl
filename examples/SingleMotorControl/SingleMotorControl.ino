#include "SpeedControl.h"
#include <Arduino.h>

SpeedControl motor1;

void setup()
{
// initial microcontroller health indication
Serial.begin(9600);
//setting motor pin parameters
motor1.setPin(9, 8, 6, 7);
//setting Motor CPR
motor1.setCPR(3000);
// Setting the reverse polarity for the motor
motor1.setReversePolarity(false);
//setting the speed of the motor in revolution per second
motor1.setSpeed(1);
}

void loop()
{
// delay for motor to adapt to the control signal  
delay(1000);
motor1.controlLoop();
Serial.println(motor1.currentVelocity);
}
