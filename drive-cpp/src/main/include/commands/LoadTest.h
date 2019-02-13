#pragma once

#include <frc/Timer.h>
#include <frc/commands/Command.h>
#include "math/Stats.h"

/**
 * Runs a set of operations to test performance.
 */
class LoadTest : public frc::Command {
 private:
  bool previousBelow;
  int loopCnts;
  double runSecs;
  frc::Timer timer;
  Stats stats;

  double runTest(int cnt);

 public:
  LoadTest();

 protected:
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
};
