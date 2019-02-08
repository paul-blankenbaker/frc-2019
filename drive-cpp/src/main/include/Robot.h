#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "OI.h"
#include "commands/Performance.h"
#include "commands/drive/DriveMeasure.h"
#include "subsystems/DriveSubsystem.h"

class Robot : public frc::TimedRobot {
 public:
  static constexpr bool debug = true;
  static DriveSubsystem drive;
  static OI oi;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
  DriveMeasure m_defaultAuto;
  Performance m_myAuto;
  frc::SendableChooser<frc::Command*> m_chooser;

  // Used to monitor performance when debug is set to true
  Performance m_performance;

};
