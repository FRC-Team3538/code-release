package com.robojackets.lib;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;

public class UnitsUtils {
  public static <T extends Unit<T>> Measure<T> abs(Measure<T> measure) {
    return measure.times(Math.signum(measure.magnitude()));
  }

  public static <T extends Unit<T>> void mut_abs(MutableMeasure<T> measure) {
    measure.mut_times(Math.signum(measure.magnitude()));
  }
}
