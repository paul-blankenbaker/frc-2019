#pragma once

#include <frc/commands/Command.h>
#include "math/Stats.h"

/**
 * Command intended for tracking performance (timing) statistics.
 */
class Performance : public frc::Command {
  private:
  double m_lastSecs;
  Stats m_stats;

 public:
  Performance();

  void AddMeasurement(double timeSecs) {
    m_lastSecs = timeSecs;
    m_stats.Add(timeSecs);
  }

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
};
