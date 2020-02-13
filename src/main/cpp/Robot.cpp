/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandScheduler.h>
#include "commands/RotateColorWheel.h"

frc::Joystick m_stick{0};
frc2::JoystickButton m_button(&m_stick, 7);
double timedRobotCount=0;
double robotInitCount=0;
double robotPeriodicCount=0;
double teleopDisableCount=0;
double teleopInitCount=0;
double teleopPeriodicCount=0;

  void Robot::RobotInit() {
    robotInitCount++;
    printf("RobotInit Iter: %f\n", robotInitCount);    
  }

  void Robot::RobotPeriodic() {
    robotPeriodicCount++;
    printf("RobotPeriodic Iter: %f\n", robotPeriodicCount);
    frc2::CommandScheduler::GetInstance().Run();
  }

  void Robot::TeleopInit() {
    teleopInitCount++;
    printf("TeleopInit Iter: %f\n", teleopInitCount);

  }

  void Robot::TeleopPeriodic() {
    teleopPeriodicCount++;
    printf("TeleopPeriodic Iter: %f\n", teleopPeriodicCount);
    if (m_stick.GetRawButtonPressed(7)){
      printf("Get Raw Pressed\n");
    }
    m_button.WhenPressed(new RotateColorWheel(&m_colorwheel));
  }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif