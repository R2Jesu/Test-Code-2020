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
#include <frc/util/color.h>
#include <frc/NidecBrushless.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "frc/DriverStation.h"

#include <frc/Compressor.h> 
#include <frc/Solenoid.h>

class Robot : public frc::TimedRobot
{
public:
  // Consturctor
  std::string gameData;
  frc::Color gameColor = nun;
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

  // Solenoids 
  frc::Compressor compressorObject;
  frc::Solenoid ballPopper{0} ; 

//color sensor
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
   rev::ColorSensorV3 m_colorSensor{i2cPort};
    rev::ColorMatch m_colorMatcher;
     static constexpr frc::Color kBlueTarget = frc::Color(0.125, 0.421, 0.454);
  static constexpr frc::Color kGreenTarget = frc::Color(0.194, 0.582, 0.223);
  static constexpr frc::Color kRedTarget = frc::Color(0.483, 0.387, 0.13);
  static constexpr frc::Color kYellowTarget = frc::Color(0.312, 0.564, 0.124);
  static constexpr frc::Color nun = frc::Color(0,0,0);
  static constexpr frc::Color Default = frc::Color(1,1,1);

   //color wheel motor
    double NidecValue=0;
  frc::NidecBrushless motor = frc::NidecBrushless(3,0);


};