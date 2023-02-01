#include "SpeedControl.h"
#include <Arduino.h>

SpeedControl motor1;

#define dirPin 9
#define pwmPin 8
#define encA 6
#define encB 7
#define p 50
#define i 30
#define d 25
#define outputMin
#define outputMax
#define CPR 3000
#define maxSpeed 2

void setup()
{
// start serial communication
Serial.begin(9600);
//setting motor related parameters
motor1.setPin(9, 8, 6, 7);
motor1.setReversePolarity(true);
//setting control parameters, These parameters are optiunal, upto the user based on the requirement
motor1.setPIDValue(p , i , d );
motor1.setPIDOutput(outputMin ,outputMax);
motor1.setCPR(CPR)
motor1.setMotorMaxSpeed(maxSpeed);


// Setting Motor speed
motor1.setSpeed(1);

}

void loop()
{
// delay to compensate the motor inertia, it varies from motor to motor and also on the CPR
delay(1000);
// Motor Control Loop
motor1.controlLoop();
// Printing the motor velocity
Serial.println(motor1.currentVelocity);

}
