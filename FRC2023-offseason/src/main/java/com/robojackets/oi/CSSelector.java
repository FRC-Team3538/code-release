package com.robojackets.oi;

import edu.wpi.first.wpilibj.RobotBase;

public class CSSelector {
  private CSSelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    return false;
  }

  /**
   * Instantiates and returns an appropriate handheld OI object (driver and operator) based on the
   * connected joysticks.
   */
  public static ControlScheme findMainControls() {
    if (RobotBase.isSimulation()) {
      return new SimCS(0, 1);
    }

    return new RealCS(0, 1);
  }
}
