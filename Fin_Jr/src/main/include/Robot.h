/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/SparkMax.h"
#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>

class Robot : public frc::TimedRobot
{
public:
  // Consturctor
  Robot();

  // =================================================
  //  Top Level Robot Functions
  // =================================================
  void RobotInit();

  void AutonomousInit();

  void AutonomousPeriodic();

  void TeleopInit();

  void TeleopPeriodic();

  void TestPeriodic();

private:
  // =================================================
  //  Class Objects
  // =================================================

  // User Control
  frc::Joystick m_stick{0};
  frc::Joystick m_OperatorStick{1};

  // Robot drive system
  rev::SparkMax m_leftMotor{0};
  rev::SparkMax m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Encoders
  frc::Encoder m_encL{7, 8, false, frc::Encoder::k4X};
  frc::Encoder m_encR{4, 5, false, frc::Encoder::k4X};

  // Shooter subsystem
  rev::CANSparkMax m_leftLeadMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{3, rev::CANSparkMax::MotorType::kBrushless};

  // Spare Motor Controller
  rev::SparkMax m_SpareMotor{2};
};