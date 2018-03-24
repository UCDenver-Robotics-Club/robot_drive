#include <vectormath.h> // libs for vector math
#include <PID_v1.h> // pid control 
#include <PRIZM.h> // for motor and servo drive 
#include <i2cdevice.h> // lib for devices 
// imu devices 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// constents 
const int STARTINTSPEED = 40; // worked pretty well @ 40
const float kP = 8; // this is found by trial and error 

// objects for running the system
PRIZM prizm; // prizm object 
Adafruit_BNO055 bno = Adafruit_BNO055(); // IMU object
Ledblink blinker(8); // led blinker for status 
// vectors for navigation
NavVector pos(0,0); // inital position
NavVector motorPower(0,0);



void setup()
{
  pinMode(13,OUTPUT);
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

  blinker.on();
  delay(250);
  blinker.off();
  delay(1000); // give enough time to run infrount of the robot
  blinker.on(); // turn led on to say we've started testing
  // testing stuff ...
  driveDistance(7,STARTINTSPEED,true); // drive 7 feet

  blinker.off(); // turn led off to say we're done testing
}

void loop()
{
  //blinker.on();
  //delay(1000);
  //blinker.off();
  //delay(1000);  
}

void drive(double correction,double rspeed)
{
  // correction for left motor
  double tempLeft = rspeed;
  // apply the correction and then normalise it.
  tempLeft += correction;
  tempLeft = normalise(tempLeft);
  //prizm.setMotorPower(2,tempLeft); // set the speed of the left motor, I think

  // correction for right motor
  double tempRight = correction;
  tempRight += rspeed;// * -1.0; 
  tempRight = normalise(tempRight); // normalise with in bounds
  //prizm.setMotorPower(1,tempRight); // set the motor speed
  
  prizm.setMotorPowers(tempLeft,tempRight);
  
  // send the motor values back to the PC
  Serial.print(" right motor: ");
  Serial.print(tempRight);
  Serial.print(" left motor: ");
  Serial.print(tempLeft);
  Serial.print(" right encoder: ");
  Serial.print(prizm.readEncoderCount(2));
  Serial.print(" left encoder: ");
  Serial.print(prizm.readEncoderCount(1));
  Serial.print(" current angle: ");
  Serial.print(getRotation());
  Serial.print(" correcting value: ");
  Serial.print(correction);
  Serial.println();
}

// drive a fixed distance, use feet if nessary
void driveDistance(float distance,float rspeed,bool usefeet)
{
  // ticks to travel and the ticks that we've already travled
  float ticksToTravel; 
  int ticksPerCm = 40; // 40 ticks per centimeter

  //prizm.setMotorPower(2,-20);

  if(usefeet)
  {
    // if the input in feet then convert to centimeters
    distance = distance * 30.08; // apply a conversion factor from feet to centimeters 
    Serial.println(distance);
  }
  // apply a conversion factor to convert from centimeters to ticks 
  ticksToTravel = distance * 40;

  // reset the encoders and the gyro scope 
  prizm.resetEncoders();
  if(!bno.begin())
  {
    Serial.println("something went wrong with the IMU while reseting in the drive distance function");
  }

  Serial.println(ticksToTravel);

  while(abs(prizm.readEncoderCount(1)) < ticksToTravel && abs(prizm.readEncoderCount(2)) < ticksToTravel)
  {
    // use proportinal feedback from the IMU to make sure that we're driving in a strait line
    float angle = getRotation();
    
    drive(angle*kP,rspeed);
    delay(5); // delay of 5ms (subject to change)
  }
  
  // when we're done turn the motors off
  prizm.setMotorSpeeds(0,0);
}

// force a value to be with in a range of [-1.0,1.0]
float normalise(float value)
{
  if(value < -100)
    return -100;
  else if(value > 100)
    return 100;
  else
    return value;
}


float getRotation()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

