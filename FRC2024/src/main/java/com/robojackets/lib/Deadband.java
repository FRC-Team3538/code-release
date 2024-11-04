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
    return apply(signal, 1);
  }

  public Vector<N2> apply(Vector<N2> signal, double pow) {
    var norm = signal.norm();

    if (norm < 1e-9) {
      return signal;
    }

    var new_magnitude = apply(norm);

    return signal.times(Math.pow(new_magnitude, pow)).div(norm);
  }
}
