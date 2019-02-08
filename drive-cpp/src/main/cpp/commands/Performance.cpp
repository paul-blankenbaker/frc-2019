#include "commands/Performance.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"

Performance::Performance() : m_lastSecs(0),
                             m_stats() {
  m_lastSecs = 0;
}

// Called just before this Command runs the first time
void Performance::Initialize() {
  m_stats.Zero();
  m_lastSecs = 0;
}

// Called repeatedly when this Command is scheduled to run
void Performance::Execute() {
  frc::SmartDashboard::PutNumber("Run Last", m_lastSecs);
  frc::SmartDashboard::PutNumber("Run Avg", m_stats.GetAvg());
  frc::SmartDashboard::PutNumber("Run Max", m_stats.GetMax());
}

// Make this return true when this Command no longer needs to run execute()
bool Performance::IsFinished() {
  return false;
}
