package com.robojackets.oi;

import com.robojackets.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlModeManager extends SubsystemBase {
  private ControlMode currentMode = Constants.DEFAULT_CONTROL_MODE;

  public ControlModeManager() {
    setDefaultCommand(goToMode(Constants.DEFAULT_CONTROL_MODE));
  }

  public Trigger isMode(ControlMode mode) {
    return new Trigger(() -> mode.equals(currentMode));
  }

  public Command goToMode(ControlMode mode) {
    return run(() -> currentMode = mode)
        .ignoringDisable(true)
        .withName(String.format("ControlModeManager::Mode(%s)", mode));
  }

  public ControlMode getCurrentMode() {
    return currentMode;
  }
}
