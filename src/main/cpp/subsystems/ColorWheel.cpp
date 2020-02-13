#include "subsystems/ColorWheel.h"

ColorWheelSubsystem::ColorWheelSubsystem() {}

void ColorWheelSubsystem::RotateColorWheel() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    m_colorMatcher.AddColorMatch(Ball);
    motor.Enable();
    int colorCount = 0;
    int colorCount2; 
  
    NidecValue=.25;
    motor.Set(NidecValue);
      
    frc::Color detectedColor = m_colorSensor.GetColor();

    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);  
    frc::Color startingColor = matchedColor;
  
    while (colorCount < 9) {
      
      detectedColor = m_colorSensor.GetColor();
      confidence = 0.0;
      matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

      if (matchedColor == startingColor) {
        if (colorCount < 8){
          colorCount2 = 0;
          while (colorCount2 < 1) {

            detectedColor = m_colorSensor.GetColor();
            confidence = 0.0;
            matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

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
    motor.Disable();
}

void ColorWheelSubsystem::GoToColor() {
  //some code here
}