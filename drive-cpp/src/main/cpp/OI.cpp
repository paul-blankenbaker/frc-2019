#include "OI.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/drive/DriveMeasure.h"

using namespace frc;

OI::OI() : driver(0) {
}

void OI::installCommands() {
}

void OI::setupDriverCommands() {
}

void OI::setupDashboardCommands() {
  SmartDashboard::PutData("Measure", new DriveMeasure());
}