// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  
try
  {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi:
     * - Communication via USB.
     * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    ahrs = new AHRS(SPI::Port::kMXP);
  }
  catch (std::exception &ex)
  {
    std::string what_string = ex.what();
    std::string err_msg("Error instantiating navX MXP:  " + what_string);
    const char *p_err_msg = err_msg.c_str();
    DriverStation::ReportError(p_err_msg);
  }

   // Init Robot Drive
  m_robotDrive.SetExpiration(0.1);
  m_drvL = 0.0;
  m_drvR = 0.0;

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{
  if (!ahrs)
    return;


  // Reset NavXP
  if ( m_DrvStickL.GetRawButton(1))
  {
    ahrs->ZeroYaw();
    ahrs->ResetDisplacement ();
  }

  // Output NavXP Data
  SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
  SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
  SmartDashboard::PutNumber("IMU_Pitch", ahrs->GetPitch());
  SmartDashboard::PutNumber("IMU_Roll", ahrs->GetRoll());
  SmartDashboard::PutNumber("IMU_CompassHeading", ahrs->GetCompassHeading());
 
  /* These functions are compatible w/the WPI Gyro Class */
  SmartDashboard::PutNumber("IMU_TotalYaw", ahrs->GetAngle());
  SmartDashboard::PutNumber("IMU_YawRateDPS", ahrs->GetRate());

  SmartDashboard::PutNumber("IMU_Accel_X", ahrs->GetWorldLinearAccelX());
  SmartDashboard::PutNumber("IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
  SmartDashboard::PutBoolean("IMU_IsMoving", ahrs->IsMoving());
 
  SmartDashboard::PutNumber("Velocity_X", ahrs->GetVelocityX());
  SmartDashboard::PutNumber("Velocity_Y", ahrs->GetVelocityY());
  SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
  SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());

  /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
  /* NOTE:  These values are not normally necessary, but are made available   */
  /* for advanced users.  Before using this data, please consider whether     */
  /* the processed data (see above) will suit your needs.                     */

  SmartDashboard::PutNumber("RawGyro_X", ahrs->GetRawGyroX());
  SmartDashboard::PutNumber("RawGyro_Y", ahrs->GetRawGyroY());
  SmartDashboard::PutNumber("RawGyro_Z", ahrs->GetRawGyroZ());
  SmartDashboard::PutNumber("RawAccel_X", ahrs->GetRawAccelX());
  SmartDashboard::PutNumber("RawAccel_Y", ahrs->GetRawAccelY());
  SmartDashboard::PutNumber("RawAccel_Z", ahrs->GetRawAccelZ());
 
  // Process Robot Drive
  R2Jesu_ProcessDrive();

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::R2Jesu_ProcessDrive(double p_LimitFactor)
{

  double LinX = m_DrvStickL.GetX();
  double LinY = m_DrvStickL.GetY();
  double LinT = m_DrvStickL.GetTwist();
  double RinX = m_DrvStickR.GetX();
  double RinY = m_DrvStickR.GetY();
  double RinT = m_DrvStickR.GetTwist();

  double VarTwistLim = 0.0;
  double curDrvL;

  frc::SmartDashboard::PutNumber("Lx", LinX);
  frc::SmartDashboard::PutNumber("Ly", LinY);
  frc::SmartDashboard::PutNumber("Lt", LinT);
  frc::SmartDashboard::PutNumber("Rx", RinX);
  frc::SmartDashboard::PutNumber("Ry", RinY);
  frc::SmartDashboard::PutNumber("Rt", RinT);

//  const double TwistLimtFactor = 0.4;
  // Values goes -1.0 to 1.0.  So shift by -1.0 to make positive 
  // and divide by 2.0 to make range 0.0 to 1.0 
  double PwrLimFactor    = -(m_DrvStickL.GetThrottle()-1.0)/2.0;  // Left Joystick
  double TwistLimtFactor = -(m_DrvStickR.GetThrottle()-1.0)/2.0;  // Right Joystick


  // Limit the drive motors for certain operations like color wheel
  if (m_DrvStickL.GetRawButton(1)) // Break or well power off
  {
    m_drvL = 0.0;
    m_drvR = 0.0;
  }
  else if (m_DrvStickL.GetRawButton(12))
  {
    CurrDriveConfig = CurrDriveConfig + 1;
    if (CurrDriveConfig == LAST_ENTRY)
        CurrDriveConfig = Std_Arcade;
  }
  else
  {   
    switch (CurrDriveConfig)
    {
    case Std_Arcade: // Single Joystick with No Twist
        m_drvL = -RinY * PwrLimFactor;  // Speed
        m_drvR =  RinX * PwrLimFactor;  // Turn
        frc::SmartDashboard::PutString("DrvMode: ","Std Arcade       ");
        break;
    case Std_Tank: // Dual   Joystick with No Twist
        m_drvL = -LinY * PwrLimFactor;  // Left Power
        m_drvR = -RinY * PwrLimFactor;  // Right Power
       frc::SmartDashboard::PutString("DrvMode: ","Std Tank         ");
       break;
    case Arcade_Twist: // Single Joystick with    Twist and Turn
        m_drvL = -RinY * PwrLimFactor;                            // Speed
        m_drvR = (RinX * PwrLimFactor) + (RinT*TwistLimtFactor);  // Turn
        frc::SmartDashboard::PutString("DrvMode: ","Arcade with Twist");
        break;
    case Dual_Arc_Twist: // Dual   Joystick with    Twist
        m_drvL = -LinY * PwrLimFactor;                            // Speed
        m_drvR = (RinX * PwrLimFactor) + (RinT*TwistLimtFactor);  // Turn
        frc::SmartDashboard::PutString("DrvMode: ","Dual Arcade Twist");
        break;
    case Arcade_Twist_Only: // Single Joystick with    Twist only
        m_drvL = -RinY * PwrLimFactor;    // Speed
        m_drvR = (RinT*TwistLimtFactor);  // Turn
        frc::SmartDashboard::PutString("DrvMode: ","Arcade Twist Only");
        break;
    case ATO_LinTurn_Limit: // Single Joystick with    Twist only and limit the turn
        m_drvL = -RinY * PwrLimFactor;             // Speed
        curDrvL = -RinY;  // Grab the current speed request
        if (fabs(curDrvL) < 0.5)   
            VarTwistLim = 0.0;  // Don't do anything between 0..+/-0.5
        else
            VarTwistLim = curDrvL/2.0;  // Thake the power and scale it.  Keep the sign
        // First remove some of the twist power based on FWD power and then
        // scale the limit factor
        m_drvR = limit(RinT-VarTwistLim)*TwistLimtFactor;  // Turn
        frc::SmartDashboard::PutString("DrvMode: ","Arcade Twist Only");
        break;
    default:
        m_drvL = 0.0;
        m_drvR = 0.0;
        frc::SmartDashboard::PutString("DrvMode: ","ERROR            ");
        break;
    }
  }

   // Drive with arcade style (use right stick)
   if (CurrDriveConfig == Std_Tank)
       m_robotDrive.TankDrive(m_drvL, m_drvR);  // Left Pwr, Right Pwr
   else
       m_robotDrive.ArcadeDrive(m_drvL, m_drvR);  // Speed, Rotate

#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutNumber("CurrDrvCfg", CurrDriveConfig);
  frc::SmartDashboard::PutNumber("DrvL", m_drvL);
  frc::SmartDashboard::PutNumber("DrvR", m_drvR);
  frc::SmartDashboard::PutNumber("PwrLimit", PwrLimFactor);
  frc::SmartDashboard::PutNumber("Twist", m_drvR);
  frc::SmartDashboard::PutNumber("TwistLimit", TwistLimtFactor);
  frc::SmartDashboard::PutNumber("VarTwistLim", VarTwistLim);
#endif
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
