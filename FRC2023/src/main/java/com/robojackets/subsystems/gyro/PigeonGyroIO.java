package com.robojackets.subsystems.gyro;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenixpro.StatusSignalValue;
import com.robojackets.lib.UnitConversion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

public class PigeonGyroIO extends GyroIO {
  private final PigeonIMU imu;

  public PigeonGyroIO(PigeonIMU imu) {
    this.imu = imu;
  }

  @Override
  public boolean isOnCanivore() {
    return false;
  }

  @Override
  public void reset() {}

  public List<StatusSignalValue<Double>> getCriticalSignals() {
    return List.of();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawRadians = -imu.getYaw() / UnitConversion.DEGREES_PER_RADIAN;

    if (inputs.quaternion == null) {
      inputs.quaternion = new double[4];
    }

    imu.get6dQuaternion(inputs.quaternion);

    // pigeon acceleromter measurements peak at +-1g, useless
    if (inputs.accel == null) {
      inputs.accel = new double[3];
    }
  }

  @Override
  public void SimPeriodic(Rotation3d delta) {
    imu.getSimCollection().addHeading(delta.getZ() * UnitConversion.DEGREES_PER_RADIAN);
  }
}
