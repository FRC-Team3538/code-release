package com.robojackets.lib;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class Deadband {
  private final double min;
  private final double max;

  public Deadband(double min, double max) {
    this.min = min;
    this.max = max;
  }

  public double apply(double signal) {
    var sgn = Math.signum(signal);
    var abs = Math.abs(signal);

    if (abs < min) {
      abs = min;
    } else if (abs > max) {
      abs = max;
    }

    return sgn * (abs - min) / (max - min) * max;
  }

  public Vector<N2> apply(Vector<N2> signal) {
    var norm = signal.norm();

    var new_magnitude = apply(norm);

    return signal.times(new_magnitude).div(norm);
  }
}
