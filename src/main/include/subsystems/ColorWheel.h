#pragma once

#include <frc/util/color.h>
#include <frc/NidecBrushless.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc2/command/SubsystemBase.h>


class ColorWheelSubsystem : public frc2::SubsystemBase {
 public:
  ColorWheelSubsystem();

  // Subsystem methods go here.

  void RotateColorWheel();

  void GoToColor();

 private:
  
    /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  double NidecValue=0;
  frc::NidecBrushless motor = frc::NidecBrushless(0,0);

  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.12, 0.42, 0.45);
  static constexpr frc::Color kGreenTarget = frc::Color(0.16, 0.58, 0.25);
  static constexpr frc::Color kRedTarget = frc::Color(0.52, 0.35, 0.13);
  static constexpr frc::Color kYellowTarget = frc::Color(0.31, 0.56, 0.12);
  static constexpr frc::Color Ball = frc::Color(0.328, .577, .093);  
};