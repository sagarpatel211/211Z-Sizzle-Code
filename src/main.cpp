/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel                                                */
/*    Created:      Sat Mar 21 2020                                           */
/*    Description:  Tower Takeover Sizzle Code (with PID)                     */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RightBackDrive       motor         17              
// RightFrontDrive      motor         18              
// LeftBackDrive        motor         14              
// LeftFrontDrive       motor         20              
// TrayTilter           motor         15              
// IntakeArm            motor         1               
// RightIntake          motor         13              
// LeftIntake           motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
vex::competition    Competition;
/*-----------------------------Broken Ports----------------------------------*/
/*Ports: 1, 3, 7, 11, 15, 16, 19                                             */
/*-------------------------------Variables-----------------------------------*/
double ColumnTarget = 200;
double ColumnDesired;
double ColumnKP = 0.0;
double ColumnPreviousError = 0.0;
double ColumnDerivative = 0.0;
double ColumnKD = 0.0;
double ColumnIntegral = 0.0;
double ColumnKI = 0.0;
double ColumnError;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
    vexcodeInit();
    Brain.resetTimer();
    RightFrontDrive.setPosition(0, degrees);
    RightBackDrive.setPosition(0, degrees);
    LeftBackDrive.setPosition(0, degrees);
    LeftFrontDrive.setPosition(0, degrees);
    TrayTilter.setPosition(0, degrees);
    IntakeArm.setPosition(0, degrees);
    LeftIntake.setPosition(0, degrees);
    RightIntake.setPosition(0, degrees);
}
/*---------------------------------------------------------------------------*/
/*                                   PID                                     */
/*---------------------------------------------------------------------------*/
double kP = 0.01; //Values Not Tuned Yet
double kD = 0.000005; //Values Not Tuned Yet
double turnkP = 0.01; //Values Not Tuned Yet
double turnkD = 0.000005; //Values Not Tuned Yet

int desiredValue = 200;
int desiredTurnValue = 0;

int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;
bool enableDrivePID = true;

int drivePID(){
  while(enableDrivePID){
    if (resetDriveSensors){
      resetDriveSensors = false;
      LeftBackDrive.setPosition (0,degrees);
      LeftFrontDrive.setPosition (0,degrees);
      RightBackDrive.setPosition (0,degrees);
      RightFrontDrive.setPosition (0,degrees);
    }
    int LeftBackPosition = LeftBackDrive.position(degrees);
    int LeftFrontPosition = LeftFrontDrive.position(degrees);
    int RightBackPosition = RightBackDrive.position(degrees);
    int RightFrontPosition = RightFrontDrive.position(degrees);
    
    double averagePosition = (LeftBackPosition+LeftFrontPosition+RightBackPosition+RightFrontPosition)/4;
    error = desiredValue - averagePosition;
    derivative = error - prevError;
    double lateralMotorPower = error * kP + derivative * kD;

    double turnDifference = ((LeftBackPosition + LeftFrontPosition)/2) - ((RightBackPosition + RightFrontPosition)/2);
    turnError = desiredTurnValue - turnDifference;
    turnDerivative = turnError - turnPrevError;
    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD; 

    LeftBackDrive.spin(fwd,(lateralMotorPower - turnMotorPower),voltageUnits::volt);
    LeftFrontDrive.spin(fwd,(lateralMotorPower - turnMotorPower),voltageUnits::volt);
    RightBackDrive.spin(fwd,(lateralMotorPower + turnMotorPower),voltageUnits::volt);
    RightFrontDrive.spin(fwd,(lateralMotorPower + turnMotorPower),voltageUnits::volt);
    
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  vex::task DriveBasePID(drivePID);
  resetDriveSensors = true;
  desiredTurnValue = 600;
  vex::task::sleep(1000);
  resetDriveSensors = true;
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    enableDrivePID = false;
    //Driver Joystick Controls (Lines 118 - 121)
    LeftBackDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() + (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
    LeftFrontDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() + (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
    RightBackDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() - (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
    RightFrontDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() - (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
    if(Controller1.ButtonB.pressing()) { //Tray Forward Button
        ColumnDesired = 880; //The desired value changes and the kP value is tuned to our requirement
        ColumnKP = 0.000000093;
        ColumnKD = 0.00;
        ColumnKI = 0.0000000;
    }
    else if(Controller1.ButtonA.pressing()) { //Tray Backward Button
        ColumnDesired = 0; //The desired value changes and the kP value is tuned to our requirement
        ColumnKP = 1;
        ColumnKD = 0;
        ColumnKI = 0;
    }
    else {
        ColumnDesired = TrayTilter.rotation(rotationUnits::deg); //The current position is desired so it doesn't move
        ColumnKP = 0;
        ColumnKD = 0;
        ColumnKI = 0;
        TrayTilter.stop(vex::brakeType::hold); //This is a backup stop just in case

    }
    ColumnError = pow((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired),3); //This calculates the error from desired value and current value
    ColumnDerivative = ColumnError - ColumnPreviousError; //This find the difference between the current error and previous error 
    if (((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired) < 5.0) && ((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired) > -5.0)){
        ColumnIntegral = 0; //We don't need integral if the error is within +/- 5
    }
    else { //If It isn't within +/- 5
        ColumnIntegral += ColumnError; 
    }
    TrayTilter.spin(reverse,(ColumnError * ColumnKP + ColumnDerivative * ColumnKD + ColumnIntegral * ColumnKI), voltageUnits::volt); //This calculates the voltage needed for the tray to move
    if(Controller1.ButtonR1.pressing()) { //Raises the intake arm
        IntakeArm.startRotateTo(400,rotationUnits::deg,100,velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing()) { //Lowers the intake arm
        IntakeArm.startRotateTo(0,rotationUnits::deg,100,velocityUnits::pct);
    }
    else {
        IntakeArm.stop(vex::brakeType::hold); //stopping the intake arm
    }
    if(Controller1.ButtonL1.pressing()) { //Button for intaking cubes
        RightIntake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        LeftIntake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing()) { //Button for outaking cubes
        RightIntake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
        LeftIntake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);    
    }
    else {
        RightIntake.stop(vex::brakeType::brake); //stopping the intakes
        LeftIntake.stop(vex::brakeType::brake);  //stopping the intakes
    }
    if(Controller1.ButtonDown.pressing()) { //Placing Stack Macro
      LeftBackDrive.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
      LeftFrontDrive.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
      RightBackDrive.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
      RightFrontDrive.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);  
      RightIntake.spin(vex::directionType::rev,80,vex::velocityUnits::pct);
      LeftIntake.spin(vex::directionType::rev,80,vex::velocityUnits::pct); 
      }
    ColumnPreviousError = ColumnError; //Updates the variables
    vex::task::sleep(20); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton();
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(20); //Slight delay so the Brain doesn't overprocess
    }
}