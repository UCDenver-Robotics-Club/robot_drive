/*
 * line sensors on page 150 of api manual 
 * 
 */

// needed for the chasses 
#include <movestack.h>
#include <move.h>
#include <robotnav.h>
#include <PRIZM.h>
#include <assert.h>
// needed for the gyroscope 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

PRIZM prizm; // object for driving the prizm chassis 
move_stack moves; // stack to keeping track of moves that the robot has made 
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() 
{
  // init the prizim lib
  prizm.PrizmBegin();
  prizm.setMotorInvert(1,1);
  // setup the move stack 
  init_stack(&moves);
  // start the serial
  Serial.begin(9600);
  Serial.println("device started");
}

void loop() 
{
  
}

// rotate and drive a distance 
void drive_vec(vmove& path)
{
  // get an angle from the gyroscope and rotate to match that angle
  // figure out how many encoder ticks its going to be to 
}

// return rotation of the IMU on the x axis 
float getAngle()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

// sets up ctc interupt 
void setup_interupts()
{
  
}

