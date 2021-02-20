/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"


double Robot::turning;
double Robot::currentDistance;

void Robot::RobotInit()
{
  // Init Timer
  m_timer.Start();

  // Init Robot Drive
  m_robotDrive.SetExpiration(0.1);
  m_drvL = 0.0;
  m_drvR = 0.0;

// Init Shooter Drive
#if R2JESU_TURNON_SHOOTER
  m_ShooterMotorLeft.RestoreFactoryDefaults();
  m_ShooterMotorRight.RestoreFactoryDefaults();
  //  Invert the right motor
  m_ShooterMotorRight.SetInverted(true);
#endif

// Init Winch Drive
#if R2JESU_TURNON_WINCH
  m_winchMotor.Set(0.0);
#endif

// Init Intake Drive
#if R2JESU_TURNON_INTAKE
  snowMotor.Set(0.0);
#endif

  //  Init Color Sensor
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
  m_colorMatcher.AddColorMatch(Default);

  // Run Pneumatics
#if R2JESU_TURNON_PNEUMATICS
  // Set Compressor Object for automatic closed loop control
  compressorObject.SetClosedLoopControl(true);
  // Set Solenoids to iniital stat
  ballPopper.Set(false);
#endif

// NavX Sensor
#if R2JESU_TURNON_NAV
  ahrs = new AHRS(frc::SPI::Port::kMXP);
  ahrs->ZeroYaw();
#endif

  // Vision & Camera Init

  // Drive USB Camera - 1
  cs::UsbCamera drvCamera = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  drvCamera.SetResolution(320, 240);
  drvCamera.SetExposureAuto();
//  drvCamera.SetFPS(15);

}

void Robot::RobotPeriodic(){

/* frc::SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
frc::SmartDashboard::PutNumber("IMU_Pitch", ahrs->GetPitch());
frc::SmartDashboard::PutNumber("IMU_Roll", ahrs->GetRoll());
frc::SmartDashboard::PutNumber("IMU_CompassHeading", ahrs->GetCompassHeading());
frc::SmartDashboard::PutNumber("IMU_Accel_X", ahrs->GetWorldLinearAccelX());
frc::SmartDashboard::PutNumber("IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
frc::SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
frc::SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());
frc::SmartDashboard::PutNumber("RawGyro_X", ahrs->GetRawGyroX());
frc::SmartDashboard::PutNumber("RawGyro_Y", ahrs->GetRawGyroY());
frc::SmartDashboard::PutNumber("RawGyro_Z", ahrs->GetRawGyroZ()); */

#if R2JESU_TURNON_VISION
   R2Jesu_Limelight();
#endif

}

void Robot::TeleopInit()
{
  gameColor = nun;
  m_ShooterMotorLeft.Set(0.0);
  m_ShooterMotorRight.Set(0.0);
}

void Robot::TeleopPeriodic()
{

  // Set the target color
  R2Jesu_CheckGameTargetColor();

  // Process intake motor commands
  R2Jesu_ProcessIntake();

  // Process user control before drive control.
  R2Jesu_ProcessUserControl();

  // Note this needs to come last to merge other drive motor request.
  R2Jesu_ProcessDrive();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
