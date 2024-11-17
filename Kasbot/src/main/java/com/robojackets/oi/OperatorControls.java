package com.robojackets.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Preferences;

public class OperatorControls {
  private final PS4Controller operator;

  public OperatorControls(PS4Controller operator) {
    this.operator = operator;

    Preferences.initDouble("Controller/Operator/Pivot_Deadband", 0);
    Preferences.initDouble("Controller/Operator/Pivot_Limit", 1);
  }

  public double pivot() {
    var deadband = Preferences.getDouble("Controller/Operator/Pivot_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Operator/Pivot_Limit", 1);

    double output = operator.getLeftY();

    return MathUtil.applyDeadband(output, deadband, limit);
  }
}
