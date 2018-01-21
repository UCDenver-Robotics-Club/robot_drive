/*
 * line sensors on page 150 of api manual 
 * 
 */

#include <movestack.h>
#include <move.h>
#include <robotnav.h>
#include <PRIZM.h>
#include <assert.h>

PRIZM prizm; // object for driving the prizm chassis 
move_stack moves; // stack to keeping track of moves that the robot has made 

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
  
}

