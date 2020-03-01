/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/math>

#include "Robot.h"

Robot::Robot() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TestPeriodic() {}

void Robot::RobotInit()
{
  /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
  m_leftLeadMotor.RestoreFactoryDefaults();
  m_rightLeadMotor.RestoreFactoryDefaults();
  //  Invert the right motor
  m_rightLeadMotor.SetInverted(true);

  // m_leftFollowMotor.RestoreFactoryDefaults();
  // m_rightFollowMotor.RestoreFactoryDefaults();

  /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
  // m_leftFollowMotor.Follow(m_leftLeadMotor);
  // m_rightFollowMotor.Follow(m_rightLeadMotor);

  /* Defines the number of samples to average when determining the rate.
     * On a quadrature encoder, values range from 1-255; larger values result in
     * smoother but potentially less accurate rates than lower values.
     */
  m_encL.SetSamplesToAverage(5);
  m_encR.SetSamplesToAverage(5);

  /* Defines how far the mechanism attached to the encoder moves per pulse. In
     * this case, we assume that a 360 count encoder is directly attached to a 3
     * inch diameter (1.5inch radius) wheel, and that we want to measure
     * distance in inches.
     */

  // MLH: Change 360 to 2048 counts per rev and assume a 1.0" wheel
  m_encL.SetDistancePerPulse(1.0 / 2048.0 * 2.0 * wpi::math::pi * 1.0);
  m_encR.SetDistancePerPulse(1.0 / 2048.0 * 2.0 * wpi::math::pi * 1.0);

  /* Defines the lowest rate at which the encoder will not be considered
     * stopped, for the purposes of the GetStopped() method. Units are in
     * distance / second, where distance refers to the units of distance that
     * you are using, in this case inches.
     */
  m_encL.SetMinRate(1.0);
  m_encR.SetMinRate(1.0);
}

void Robot::TeleopPeriodic()
{

  // Drive with arcade style
  m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());

  // Retrieve the net displacement of the Encoder since the last Reset.
  frc::SmartDashboard::PutNumber("EncL Distance", m_encL.GetDistance());
  frc::SmartDashboard::PutNumber("EncL Cout", m_encL.Get());
  frc::SmartDashboard::PutNumber("EncL Rate", m_encL.GetRate());
  frc::SmartDashboard::PutNumber("EncR Distance", m_encR.GetDistance());
  frc::SmartDashboard::PutNumber("EncR Cout", m_encR.Get());
  frc::SmartDashboard::PutNumber("EncR Rate", m_encR.GetRate());

  // Control shooter
  double l_Y = m_OperatorStick.GetY();
  m_leftLeadMotor.Set(l_Y);
  m_rightLeadMotor.Set(l_Y);

  frc::SmartDashboard::PutNumber("MtrPWR", l_Y);
  frc::SmartDashboard::PutNumber("MtrSPD_L", m_leftLeadMotor.Get());
  frc::SmartDashboard::PutNumber("MtrSPD_R", m_rightLeadMotor.Get());

  // Run Spare Motor
  m_SpareMotor.Set(m_OperatorStick.GetX());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
