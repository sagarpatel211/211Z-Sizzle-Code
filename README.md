# 211Z-Sizzle-Code
This is the VEX Tower Takeover (2019 - 2020) code for 211Z in VexCode Pro V5 Text! It includes pre-autonomous, autonomous routine with a prototype PID autonomous function, and driver control!

This repository represents the team 211Z from Sir Winston Churchill Secondary School (St. Catharines, Ontario).


## Table of Contents
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Features](#features)
* [Contributors](#contributors)
* [Contact](#contact)


## Dependencies
[VexCode Pro V5 Text 2.0.0 or later](https://www.vexrobotics.com/vexcode-download)


## Installation
* Make sure all the dependencies are installed
* Download the files
  * Option 1: 🍴 Fork this repository!
  * Option 2: 🧪 Clone the repository to your local machine using https://github.com/sagarpatel211/211Z-Sizzle-Code.git!
* Open *2019-2020_211Z_Code.v5code* in VexCode to open the program
* Download the program to the brain by connecting the V5 Brain or controller to the device via micro-USB and select *download*. In both options, the V5 Brain must be on!
* Run the program by selecting it from the V5 Brain or pressing the *play* button in VexCode **if** the V5 Brain or controller is attached to the device via micro-USB.


## Features
* Contains all V5 Smart Motors set up
```
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
```
* Autonomous for Unprotected Red Zone
* PID For Tray Tilter and PID prototype for drive base in autonomous 
```
ColumnError = pow((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired),3); //This calculates the error from desired value and current value
ColumnDerivative = ColumnError - ColumnPreviousError; //This find the difference between the current error and previous error 
if (((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired) < 5.0) && ((TrayTilter.rotation(rotationUnits::deg) - ColumnDesired) > -5.0)){
   ColumnIntegral = 0; //We don't need integral if the error is within +/- 5
}
else { //If It isn't within +/- 5
   ColumnIntegral += ColumnError; 
}
TrayTilter.spin(reverse,(ColumnError * ColumnKP + ColumnDerivative * ColumnKD + ColumnIntegral * ColumnKI), voltageUnits::volt);
```
* Tank drive joystick/driver control 
```
LeftBackDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() + (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
LeftFrontDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() + (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
RightBackDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() - (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
RightFrontDrive.spin(vex::directionType::fwd, (Controller1.Axis3.value() - (Controller1.Axis4.value()*0.3)), vex::velocityUnits::pct);
```


## Contributors
| <a href="https://github.com/sagarpatel211" target="_blank">**Sagar Patel**</a> | <a href="http://github.com/saurinpatel20" target="_blank">**Saurin Patel**</a> |
| :---: |:---:|
| [![Sagar Patel](https://avatars1.githubusercontent.com/u/34544263?s=200)](https://github.com/sagarpatel211)    | [![Saurin Patel](https://avatars3.githubusercontent.com/u/62221622?s=200)](http://github.com/saurinpatel20) |
| <a href="https://github.com/sagarpatel211" target="_blank">`github.com/sagarpatel211`</a> | <a href="http://github.com/saurinpatel20" target="_blank">`github.com/saurinpatel20`</a> |


## Contact
[Email](mailto:sa24pate@uwaterloo.ca) | [Website](https://sagarpatel211.github.io/)
