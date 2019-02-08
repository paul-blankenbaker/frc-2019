#pragma once

#include <frc/Joystick.h>

class OI {
 private:
  // Joystick used by main driver of robot
  frc::Joystick driver;

 public:
  OI();

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

  /**
   * Install primary driver specific commands.
   */
  void setupDriverCommands();

  void setupDashboardCommands();
};
