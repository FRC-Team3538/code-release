package com.robojackets.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation3d;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLog;

public class GyroIO {
  public void updateInputs(GyroIOInputs inputs) {}

  public void reset() {}

  public boolean isOnCanivore() {
    return false;
  }

  public void SimPeriodic(Rotation3d delta) {}

  @AutoLog
  @Getter
  @Setter
  public static class GyroIOInputs {
    double yawRadians;
    double pitchRadians;
    double rollRadians;

    double[] quaternion = new double[4];
    double[] accel = new double[3];
  }
}
