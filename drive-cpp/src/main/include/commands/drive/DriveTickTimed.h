#pragma once

#include <frc/commands/Command.h>
#include "commands/drive/DriveHuman.h"

/**
 * An auton test to check how consistent your drive base is when driven purely
 * by time.
 */
class DriveTickTimed : public frc::Command {
 private:
  static constexpr double kDefaultPower = 0.4;

  enum Mode {
    rampUp,
    cruise,
    rampDown,
    done
  };

  // Labels for SmartDashboard controls
  static constexpr const char* fixedLeftLabel = DriveHuman::fixedLeftLabel;
  static constexpr const char* fixedRightLabel = DriveHuman::fixedRightLabel;
  static constexpr const char* rampTicksLabel = "Ramp Ticks";
  static constexpr const char* cruiseTicksLabel = "Cruise Ticks";

  Mode mode;

  /** Used to count ticks (executions) in each mode. */
  int ticks;

  /** How many execute runs to ramp power up/down to cruise speed. */
  int rampTicks;
  /** How many execute() runs to keep power at same value while cruising. */
  int cruiseTicks;

  // Gain multipliers to adjust left/right fixed values read from the dashboard
  double leftGain;
  double rightGain;

  // Maximum power to apply to left and right side
  double rightPower;
  double leftPower;

  /**
   * Changes current mode of execute function when tick count is reached.
   * 
   * @param newMode   The mode to transition when the tick count is reached.
   * @param atTickCnt The tick count that triggers the mode change.
   */
  void checkModeChange(Mode newMode, int atTickCnt);

 public:
 
  /**
   * Initialize object with multipliers for left/right side of drive train.
   * 
   * @param leftGain - Power multiplier/inverter for left side (typically 1.0 or -1.0).
   * @param rightGain - Power multiplier/inverter for right side (typically 1.0 or -1.0).
   */
  DriveTickTimed(double leftGain, double rightGain);

 protected:
  /**
   * Read in and apply current drive choices.
   */
  void Initialize() override;

  /**
   * Drive robot according to options specified during initialization.
   */
  void Execute() override;

  bool IsFinished() override;

  // Called once after isFinished returns true
  void End();

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  void Interrupted() override;
};
