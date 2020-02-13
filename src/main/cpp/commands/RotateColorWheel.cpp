#include "commands/RotateColorWheel.h"

RotateColorWheel::RotateColorWheel(ColorWheelSubsystem* subsystem) : m_colorwheel(subsystem) {
  AddRequirements({subsystem});
}

void RotateColorWheel::Initialize() { m_colorwheel->RotateColorWheel(); }

bool RotateColorWheel::IsFinished() { return true; }