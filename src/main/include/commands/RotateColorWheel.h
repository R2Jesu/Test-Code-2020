#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ColorWheel.h"

/**
 * A command that rotates the color wheel with the ColorWheel subsystem.  
 */
class RotateColorWheel : public frc2::CommandHelper<frc2::CommandBase, RotateColorWheel> {
 public:
  explicit RotateColorWheel(ColorWheelSubsystem* subsystem);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  ColorWheelSubsystem* m_colorwheel;
};