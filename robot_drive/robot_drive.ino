#include <vectormath.h> // libs for vector math
#include <PID_v1.h> // pid control 
#include <PRIZM.h> // for motor and servo drive 
#include <i2cdevice.h> // lib for devices 
// imu devices 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <ArduinoJson.h> 


// objects for running the system
PRIZM prizm; // prizm object 
Adafruit_BNO055 bno = Adafruit_BNO055(); // IMU object
Ledblink blinker(8); // led blinker for status 
NavVector pos(0,0); // inital position
// powerlevles for the left and right motor
int motor1,motor2;
// set points and error for driving in a strait line

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
 motor1 = 20;
 motor2 = 20;
 prizm.setMotorPowers(motor1,motor2);
  
}

void loop()
{
  Serial.println(getRotation());
  blinker.on();
  delay(200);
  blinker.off();
  delay(200);
}

float getRotation()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

