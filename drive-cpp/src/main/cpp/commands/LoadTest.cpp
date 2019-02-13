#include "commands/LoadTest.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include "Robot.h"

using namespace frc;

LoadTest::LoadTest() : Command("LoadTest") {
  previousBelow = false;
  loopCnts = 8096;
  runSecs = 0.0;
  SetRunWhenDisabled(true);
}

double LoadTest::runTest(int cnt) {
  double accum = 0.0;
  for (int i = 0; i < cnt; i++) {
    int n = (i / 4);
    for (int j = 0; j < n; j++) {
      int x = ((j >> 3) + 1) * 7 / 8;
      if ((j % 4) == 0) {
        x = -x;
      }
      stats.Add(x);
    }
    double ratio = (double)i / (double)cnt;
    accum += sin(ratio);
    accum += pow(3.1415927, ratio);
    accum += cos(ratio);
    stats.Add(accum);
  }

  return accum;
}

void LoadTest::Initialize() {
  previousBelow = false;
  runSecs = 0;
  loopCnts = 1;
  stats.Zero();
}

void LoadTest::Execute() {
  SmartDashboard::PutNumber("CPU Test Loops", loopCnts);
  timer.Reset();
  timer.Start();
  runTest(loopCnts);
  runSecs = timer.Get();
}

bool LoadTest::IsFinished() {
  if (runSecs > 0.1) {
    if (previousBelow) {
      return true;
    }
    previousBelow = false;
    loopCnts = loopCnts / 2;
  } else {
    previousBelow = true;
    loopCnts = loopCnts * 2;
  }
  return false;
}

void LoadTest::End() {
  double hz = 0;
  if (runSecs > 0) {
    hz = loopCnts / runSecs;
  }
  SmartDashboard::PutNumber("CPU Test Secs", runSecs);
  SmartDashboard::PutNumber("CPU Test Hz", hz);
}
