package frc.math;

public class Stats {
    private int cnt;
    private double sum;
    private double min;
    private double max;

    public Stats() {
        zero();
    }

    public void zero() {
        cnt = 0;
        min = max = sum = 0.0;
    }

    public void add(final double val) {
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

    public final int getCnt() {
        return cnt;
    }

    public final double getSum() {
        return sum;
    }

    public final double getMin() {
        return min;
    }

    public final double getMax() {
        return max;
    }

    public final double getAvg() {
        return (cnt != 0) ? sum / cnt : 0;
    }
}