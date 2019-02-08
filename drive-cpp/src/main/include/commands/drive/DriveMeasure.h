#pragma once

#include <frc/commands/Command.h>

/**
 * Utility command that measures drive distance and rotation (zeros each time it
 * is started).
 */
class DriveMeasure : public frc::Command {
 private:
  // Used to keep track of starting values
  double yawLast;
  double leftDistLast;
  int leftCntsLast;
  double rightDistLast;
  int rightCntsLast;

 public:
  DriveMeasure();

 protected:
  /**
   * Takes initial values of things measurements to "zero" each time the command
   * is started.
   */
  void Initialize() override;

  // Called repeatedly when this Command is scheduled to run
  void Execute() override;

  bool IsFinished() override;
};
