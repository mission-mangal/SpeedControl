#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include <Arduino.h>
#include <Encoder.h>
#include <AutoPID.h>

class SpeedControl
{

private:
  // Motor Pin Configuration
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t encA;
  uint8_t encB;
  bool polarity_reverse = false;

  //Motor Related Parameters
  double CPR = 36124;
  float motorRadius = 1; // in metres
  float maxSpeed = 2.0; // Rotation Per Second

  // PID Control Related Parameter
  double outputMin  = -255;
  double  outputMax = 255;
  float kP  = 50;
  float kI  = 30.0;
  float kD  = 20.0;

public: 
  //Velocity control related parameters
  volatile long currentPosition;
  volatile long previousPosition;
  volatile double previousTime;
  volatile double currentTime;
  double currentVelocity =  0;
  double setPoint = 0;
  double outputPWM = 0;
  // double * position;
  AutoPID pid{&currentVelocity, &setPoint, &outputPWM, outputMin, outputMax, kP, kI, kD};
  Encoder * encoder;
  
  void  initialSetup()
  {
  // Setting the pin configurations
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(dirPin , LOW);
  analogWrite(pwmPin , 0);
  // delay(2000);
  //variables for velocity calculation
  setPoint  = 0.0;
  currentPosition = 0;
  previousPosition = 0;
  previousTime = micros();
  currentVelocity = 0;
  outputPWM  = 0;
  }

  SpeedControl()
  {
    // params:  direction pin , pwm pin , encoder A , encoder B
   
  }

  void setPin(uint8_t pinDir , uint8_t pinPWM ,uint8_t aEnc , uint8_t bEnc )
  {
  dirPin = pinDir;
  pwmPin = pinPWM;
  encA  = aEnc;
  encB = bEnc;
  encoder  =  new Encoder(encA , encB);
  initialSetup();

  }

  ~SpeedControl()
  {
    // destructor for the class
    delete encoder;
  }

  void setReversePolarity(bool polarity)
  {
    polarity_reverse = polarity;
  }

  void setMotorPWM(int PWM)
  {
  if (polarity_reverse)
    PWM = PWM * -1;
    // Changing the motor direction based on the PWM value
  if (PWM < 0)
      digitalWrite(dirPin , LOW);
  else 
      digitalWrite(dirPin , HIGH);

  // generating the PWM pulses for changing the motor velocity
    analogWrite(pwmPin , abs(PWM));
  }

  void setPIDValue(float p , float i , float d )
  {
    kP = p;
    kI = i;
    kD = d;
    pid.setGains(kP, kI , kD);
  }

  void setPIDOutput(int min , int max)
  {
    outputMin = min;
    outputMax = max;
    pid.setOutputRange(outputMin , outputMax);
  }

  void setCPR(int cpr)
  {
    CPR = cpr;
  }

  void setMotorMaxSpeed(float speed)
  {
    maxSpeed = speed;
  }

  void setSpeed(float rps)
  {
    // input: motor speed in rotation per second
    if (rps < maxSpeed)
    setPoint = rps;
    else
    setPoint = maxSpeed;
  }

  void controlLoop()
  { 
  // Calculating the motor velocity
  currentPosition = encoder->read();
  currentTime = micros();
  currentVelocity = (currentPosition - previousPosition)/((CPR / 1000000.00) * (currentTime - previousTime));
  previousPosition = currentPosition;
  previousTime = currentTime;
 // Running PID and changing motor PWM signal accordingly
  pid.run();
  setMotorPWM(outputPWM);
  }


};

#endif
