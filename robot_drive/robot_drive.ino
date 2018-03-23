#include <vectormath.h> // libs for vector math
#include <PID_v1.h> // pid control 
#include <PRIZM.h> // for motor and servo drive 
#include <i2cdevice.h> // lib for devices 
// imu devices 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// constents 
const int startingSpeed = 25;


// objects for running the system
PRIZM prizm; // prizm object 
Adafruit_BNO055 bno = Adafruit_BNO055(); // IMU object
Ledblink blinker(8); // led blinker for status 
// vectors for navigation
NavVector pos(0,0); // inital position
NavVector motorPower(0,0);
// position for the robot to drive to


// powerlevles for the left and right motor
int motor1,motor2;
// set points and error for driving in a line
double setPoint = 0;
double error=0;
double correction=0;
PID directionCorrection(&error,&correction,&setPoint,1,0,0,DIRECT);

void setup()
{
  Serial.begin(9600); // start of the serial port 
  prizm.PrizmBegin(); // start up the prizm board and wait for go 

  // start up the bno and make sure that it connected
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
    delay(10000); // delay for 10 seconds so the OP can know what happend 
  }

  // setup the motors 
  prizm.setMotorInvert(1,1); // invert motor #1 
 // prizm.setServoPosition(2,0); // turn on the convair belt

  // set the initial values for the motors
  motorPower.setValues(startingSpeed,startingSpeed);
  directionCorrection.SetMode(AUTOMATIC); // turn the pid control on

  blinker.off(); // make sure that the status light is off
}

void loop()
{
//  Serial.println(getRotation());
//  blinker.on();
//  delay(100);
//  Serial.println(getRotation());
//  blinker.off();
//  delay(100);

  float theta = getRotation();
  if(theta > 5)
  {
    blinker.on();
  }
  else
  {
    blinker.off();
  }
  Serial.println(theta);

  // update the speed of the motors 
  prizm.setMotorPowers(motorPower.getX(),motorPower.getY()); // set the motor speeds based on a vector 
}

float getRotation()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

