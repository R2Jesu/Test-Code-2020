/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>
#include <frc/NidecBrushless.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
frc::Joystick m_stick{0};
double globalCount=0;
/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect various colors.
 */
class Robot : public frc::TimedRobot {
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

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
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

 public:
  void RobotInit() {
    System.out.println()
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    m_colorMatcher.AddColorMatch(Ball);  
    
  }
  void RobotPeriodic() {
    
  }
  void TeleopInit() {
    frc::SmartDashboard::PutString("Stage","Init");
    motor.Enable();

  }

  void TeleopPeriodic() {
    globalCount++;
    frc::SmartDashboard::PutString("Stage","Periodic");
    frc::SmartDashboard::PutNumber("Periodic Count", globalCount);
    if (m_stick.GetRawButton(7)){
      //run motor until green is seen 9 times
    int colorCount = 0;
    int colorCount2; 
  
    NidecValue=.25;
       motor.Set(NidecValue);
      
      frc::Color detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);  
    frc::Color startingColor = matchedColor;
  
     while (colorCount < 9) {
      
        /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    confidence = 0.0;
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } else if (matchedColor == Ball) {
      colorString = "Ball";
    } else {
      colorString = "Unknown";
    }




    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
       if (matchedColor == startingColor) {
         if (colorCount < 8){
         colorCount2 = 0;
         while (colorCount2 < 1) {
        /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    confidence = 0.0;
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } else if (matchedColor == kGreenTarget) {\
      colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } else if (matchedColor == Ball) {
      colorString = "Ball";
    } else {
      colorString = "Unknown";
    }




    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
       if (!(matchedColor == startingColor)) {
         colorCount2++;
       }
        }
         }
         colorCount++;
       }
     }
      NidecValue=0;
       motor.Set(NidecValue);
       //NidecValue=.25;
       //motor.Set(NidecValue);
    }



    
  }

  void TeleopDisable() {
       motor.Disable(); 

    }
  
 
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif