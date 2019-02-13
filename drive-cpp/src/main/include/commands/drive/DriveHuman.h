#pragma once

#include <frc/commands/Command.h>

/**
 * An example command. You can replace me with your own command.
 */
class DriveHuman : public frc::Command {
 private:
  // Drive mode choices
  static constexpr int kModeArcade = 0;
  static constexpr int kModeTank = 1;
  static constexpr int kModeCurvature = 2;
  static constexpr int kModeFixed = 3;

  static constexpr int kThrottleAxis = 1;
  static constexpr int kRotationAxis = 4;
  static constexpr int kLeftAxis = 1;
  static constexpr int kRightAxis = 3;

  bool squareInputs;

  bool quickTurn;

  int mode;

  double fixedLeft;

  double fixedRight;

  double rotationGain;

  double slowGain;

  double minDeflect;

 public:
  // Labels for SmartDashboard controls
  static constexpr const char* quickTurnLabel = "Quick Turn";
  static constexpr const char* squaredInputsLabel = "Square Inputs";
  static constexpr const char* fixedLeftLabel = "Fixed Left";
  static constexpr const char* fixedRightLabel = "Fixed Right";
  static constexpr const char* rotationGainLabel = "Rotation Gain";
  static constexpr const char* slowGainLabel = "Slow Gain";
  static constexpr const char* flippedFrontLabel = "Flipped Front";
  static constexpr const char* brakeModeLabel = "Brake Mode";
  static constexpr const char* driveModeLabel = "Drive Mode"; 

  /**
   * Initialize all of the control options and control option UI.
   */
  DriveHuman();

  static frc::Command* createFlipFrontCommand();

  static frc::Command* createBrakeModeToggleCommand();

  /**
   * Helper method that OI can use during setup to enable dashboard driver control settings.
   */
  static void setupDashboardControls();

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

  /**
   * Drives left and right side motors at fixed power value specified on the dashboard.
   * 
   * @param gain - Gain multiplier for front/back.
   * @param rotGain - Gain multiplier for rotation.
   */
  void driveFixed(double gain, double rotGain);
};
