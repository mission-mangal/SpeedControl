#include "SpeedControl.h"
#include <Arduino.h>

MotorControl frontLeft;
MotorControl frontRight;


void setup()
{
// initial microcontroller health indication
Serial.begin(9600);

//setting motor pin parameters
frontLeft.setPin(9, 8, 6, 7);
// Setting the reverse polarity for the motor
frontLeft.setReversePolarity(true);
frontRight.setPin( 4 , 5 , 2, 3);
frontRight.setReversePolarity(false);

frontLeft.setSpeed(1);
frontRight.setSpeed(1);
}

void loop()
{
delay(1000);
frontLeft.controlLoop();
frontRight.controlLoop();
Serial.println(frontLeft.currentVelocity);
Serial.println(frontRight.currentVelocity);
}