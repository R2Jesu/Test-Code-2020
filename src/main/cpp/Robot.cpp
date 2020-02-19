/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/spark.h>
#include <Robot.h>
#include <frc/Encoder.h>
#include <wpi/math>

  frc::Encoder m_encoder{7,8, true};
  frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_right{0};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stick{0};
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
  frc::Spark launchMotorLeft{9}; 
  frc::Spark launchMotorRight{8}; 

void Robot::RobotInit() {
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
    printf("Incatchblock\n");
    DriverStation::ReportError(p_err_msg);
  }
  }

  void Robot::AutonomousInit() {
    {ahrs->ZeroYaw();}
    m_encoder.Reset();
  m_encoder.SetSamplesToAverage(5); 
  m_encoder.SetDistancePerPulse(((wpi::math::pi * 6.0)/2048) / 12);
  m_encoder.SetMinRate(1.0);
  while ((m_encoder.GetDistance() < 42.75)) {
    if (ahrs->GetYaw() < 0){
    m_robotDrive.ArcadeDrive(-0.4, 0.1);
    } else if (ahrs->GetYaw() > 0){
      m_robotDrive.ArcadeDrive(-0.4, -0.1);
    } else {
      m_robotDrive.ArcadeDrive(-0.4, 0);
      }
    
    }
  while (ahrs->GetYaw() < 180){
    m_robotDrive.ArcadeDrive(0, 0.3);
   }
    m_robotDrive.ArcadeDrive(0.0, 0.0);
    m_encoder.Reset();
    {ahrs->ZeroYaw();}
     while (m_encoder.GetDistance() < 42.75) {
       if (ahrs->GetYaw() < 0){
    m_robotDrive.ArcadeDrive(-0.4, 0.1);
    } else if (ahrs->GetYaw() > 0){
      m_robotDrive.ArcadeDrive(-0.4, -0.1);
    } else {
      m_robotDrive.ArcadeDrive(-0.4, 0);
      }
     };
    }
  }

  void Robot::AutonomousPeriodic()  {
    
  }

  void Robot::TeleopInit() 

  void Robot::TeleopPeriodic()  {
    // Drive with arcade style (use right stick)
    m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());
 

    if (!ahrs)
    return;

  //bool reset_yaw_button_pressed = m_stick->GetRawButton(1);
  //if (reset_yaw_button_pressed)
  //{
  //  ahrs->ZeroYaw();
  //}

  SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
  SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
  SmartDashboard::PutNumber("IMU_Pitch", ahrs->GetPitch());
  SmartDashboard::PutNumber("IMU_Roll", ahrs->GetRoll());
  SmartDashboard::PutNumber("IMU_CompassHeading", ahrs->GetCompassHeading());
  SmartDashboard::PutNumber("IMU_Update_Count", ahrs->GetUpdateCount());
  SmartDashboard::PutNumber("IMU_Byte_Count", ahrs->GetByteCount());
  SmartDashboard::PutNumber("IMU_Timestamp", ahrs->GetLastSensorTimestamp());

  /* These functions are compatible w/the WPI Gyro Class */
  SmartDashboard::PutNumber("IMU_TotalYaw", ahrs->GetAngle());
  SmartDashboard::PutNumber("IMU_YawRateDPS", ahrs->GetRate());

  SmartDashboard::PutNumber("IMU_Accel_X", ahrs->GetWorldLinearAccelX());
  SmartDashboard::PutNumber("IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
  SmartDashboard::PutBoolean("IMU_IsMoving", ahrs->IsMoving());
  SmartDashboard::PutNumber("IMU_Temp_C", ahrs->GetTempC());
  SmartDashboard::PutBoolean("IMU_IsCalibrating", ahrs->IsCalibrating());

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
  SmartDashboard::PutNumber("RawMag_X", ahrs->GetRawMagX());
  SmartDashboard::PutNumber("RawMag_Y", ahrs->GetRawMagY());
  SmartDashboard::PutNumber("RawMag_Z", ahrs->GetRawMagZ());
  SmartDashboard::PutNumber("IMU_Temp_C", ahrs->GetTempC());
  /* Omnimount Yaw Axis Information                                           */
  /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
  AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
  SmartDashboard::PutString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
  SmartDashboard::PutNumber("YawAxis", yaw_axis.board_axis);

  /* Sensor Board Information                                                 */
  SmartDashboard::PutString("FirmwareVersion", ahrs->GetFirmwareVersion());

  /* Quaternion Data                                                          */
  /* Quaternions are fascinating, and are the most compact representation of  */
  /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
  /* from the Quaternions.  If interested in motion processing, knowledge of  */
  /* Quaternions is highly recommended.                                       */
  SmartDashboard::PutNumber("QuaternionW", ahrs->GetQuaternionW());
  SmartDashboard::PutNumber("QuaternionX", ahrs->GetQuaternionX());
  SmartDashboard::PutNumber("QuaternionY", ahrs->GetQuaternionY());
  SmartDashboard::PutNumber("QuaternionZ", ahrs->GetQuaternionZ());

  frc::SmartDashboard::PutNumber("Encoder Distance", m_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder Cout", m_encoder.Get());
  frc::SmartDashboard::PutNumber("Encoder Rate", m_encoder.GetRate());

 m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

 

    
  }


  void Robot::RobotPeriodic()  {

  }

  void Robot::TestPeriodic()  {

  }

  // Robot drive system



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif