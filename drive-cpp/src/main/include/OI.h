#pragma once

#include <frc/Joystick.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

class OI {
 private:
  static constexpr bool debug = true;

  // Joystick used by main driver of robot
  frc::Joystick driver;

  frc::SendableChooser<frc::Command*> autonChooser;

 public:
  OI();

  static bool initializeBoolean(const char* dashboardLabel, bool defaultValue);

  static double initializeNumber(const char* dashboardLabel, double defaultValue);

  frc::Joystick* getDriverJoystick() {
    return &driver;
  }

  bool readDriverButton(int buttonId) {
    return driver.GetRawButton(buttonId);
  }

  double readDriverAxis(int axisId) {
    return driver.GetRawAxis(axisId);
  }

  void installCommands();

  frc::Command* getSelectedAuton() {
    return autonChooser.GetSelected();
  }

 private:
  /**
   * Install primary driver specific commands.
   */
  void setupDriverCommands();

  void setupDashboardCommands();
};
