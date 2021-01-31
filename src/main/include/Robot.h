// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h.>
#include <frc/DriverStation.h>
#include "AHRS.h"

// Motor Controllers
#include <frc/spark.h>
#include <frc/PWMVictorSPX.h>
#include <frc/drive/DifferentialDrive.h>

using namespace frc;

#define R2JESU_TURNON_SMARTDASHBOARD 1

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  // =================================================
  //  Subsystem Control Functions
  // =================================================

  //  Main Robot Drive
 void R2Jesu_ProcessDrive(double p_LimitFactor = 1.0);

 double limit(double inVal, bool PosOnly = false)
 {
   double val = ((inVal>1.0)?1.0:((inVal<-1.0)?-1.0:inVal));
   
   if (PosOnly && (val < 0.0))
       val = 0.0;

   return(val);
 }

  // Autonomous Test Code to control from Driver Station
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // NavXP Sensor
  AHRS *ahrs;

  // Drive Configurations
  typedef enum {
    Std_Arcade        = 0, // Single Joystick with No Twist
    Std_Tank          = 1, // Dual   Joystick with No Twist
    Arcade_Twist      = 2, // Single Joystick with    Twist and Turn
    Dual_Arc_Twist    = 3, // Dual   Joystick with    Twist
    Arcade_Twist_Only = 4, // Single Joystick with    Twist only
    ATO_LinTurn_Limit = 5, // Single Joystick with    Twist only and limit the turn
    LAST_ENTRY        = 6
  } DriveConfigType;
   
  unsigned int CurrDriveConfig = ATO_LinTurn_Limit;

  // User Controllers
  frc::Joystick m_DrvStickL{0};
  frc::Joystick m_DrvStickR{1};

  // Robot drive system
  frc::PWMVictorSPX m_leftMotor{0};  // Second motor wired to Y PWM Cablec
  frc::PWMVictorSPX m_rightMotor{1}; // Second motor wired to Y PWM Cable
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Because motor control cmds may come from other parts of the code use these
  // objects to control the final motor controldouble m_drvL;
  double m_drvR;
  double m_drvL;

  int autoLoopCounter;
};
