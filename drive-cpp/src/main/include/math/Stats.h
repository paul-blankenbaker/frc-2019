/**
 * Simple helper class to track basic statisics (min, max, avg, cnt).
 */
class Stats {
 private:
  int cnt;
  double sum;
  double min;
  double max;

 public:
  Stats() {
    Zero();
  }

  void Zero() {
    cnt = 0;
    min = max = sum = 0.0;
  }

  void Add(const double val) {
    if (cnt == 0) {
      min = max = val;
    } else if (val > max) {
      max = val;
    } else if (val < min) {
      min = val;
    }
    cnt++;
    sum += val;
  }

  int GetCnt() const {
    return cnt;
  }

  double GetSum() const {
    return sum;
  }

  double GetMin() const {
    return min;
  }

  double GetMax() const {
    return max;
  }

  double GetAvg() const {
    return (cnt != 0) ? sum / cnt : 0;
  }
};