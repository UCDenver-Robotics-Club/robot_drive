// includes
#include <PID_v1.h>
#include <PRIZM.h>
#include <i2cdevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoJson.h>


// program for test driving
PRIZM prizm; // prizm object 
Adafruit_BNO055 bno = Adafruit_BNO055(); // IMU object
Ledblink blinker(8); // led blinker for status 

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
  prizm.setServoPosition(2,0); // turn on the convair belt
  
}

void loop()
{
  Serial.println(getRotation());
  
}

float getRotation()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

