package com.robojackets.subsystems.gearbox;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.controls.ControlRequest;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLog;

public class GearboxIO {

  public void setInverted(boolean inverted) {}

  public void setCurrentLimit(double amps, boolean enabled) {}

  public void setBrakeMode(boolean brake) {}

  public StatusCode setControl(ControlRequest request) {
    return StatusCode.OK;
  }

  public void updateInputs(GearboxIOInputs ioInputs) {}

  public boolean isOnCanivore() {
    return false;
  }

  public StatusCode setPosition(double position) {
    return StatusCode.OK;
  }

  @AutoLog
  @Getter
  public static class GearboxIOInputs {
    double voltage;
    double dutyCycle;

    double position;
    double velocity;

    double supplyCurrentAmps;
    double statorCurrentAmps;
    double torqueCurrentAmps;

    double temperatureCelsius;

    double closedLoopError;
  }
}
