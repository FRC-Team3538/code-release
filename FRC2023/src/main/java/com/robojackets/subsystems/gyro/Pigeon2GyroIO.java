package com.robojackets.subsystems.gyro;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.robojackets.lib.SignalManager;
import com.robojackets.lib.UnitConversion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

public class Pigeon2GyroIO extends GyroIO {
  private final Pigeon2 imu;

  private final StatusSignalValue<Double> yawSignal;
  private final StatusSignalValue<Double> pitchSignal;
  private final StatusSignalValue<Double> rollSignal;

  private final StatusSignalValue<Double> wSignal;
  private final StatusSignalValue<Double> xSignal;
  private final StatusSignalValue<Double> ySignal;
  private final StatusSignalValue<Double> zSignal;
  private final StatusSignalValue<Double> axSignal;
  private final StatusSignalValue<Double> aySignal;
  private final StatusSignalValue<Double> azSignal;

  public Pigeon2GyroIO(Pigeon2 imu) {
    this.imu = imu;

    yawSignal = imu.getYaw();
    pitchSignal = imu.getPitch();
    rollSignal = imu.getRoll();

    wSignal = imu.getQuatW();
    xSignal = imu.getQuatX();
    ySignal = imu.getQuatY();
    zSignal = imu.getQuatZ();

    axSignal = imu.getAccelerationX();
    aySignal = imu.getAccelerationY();
    azSignal = imu.getAccelerationZ();

    SignalManager.getInstance()
        .register(
            yawSignal,
            pitchSignal,
            rollSignal,
            wSignal,
            xSignal,
            ySignal,
            zSignal,
            axSignal,
            aySignal,
            azSignal);
  }

  @Override
  public boolean isOnCanivore() {
    return !imu.getCANBus().equals("rio");
  }

  @Override
  public void reset() {
    imu.reset();
  }

  public List<StatusSignalValue<Double>> getCriticalSignals() {
    return List.of(yawSignal);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawRadians = yawSignal.getValue() / UnitConversion.DEGREES_PER_RADIAN;
    inputs.pitchRadians = pitchSignal.getValue() / UnitConversion.DEGREES_PER_RADIAN;
    inputs.rollRadians = rollSignal.getValue() / UnitConversion.DEGREES_PER_RADIAN;

    if (inputs.quaternion == null) {
      inputs.quaternion = new double[4];
    }

    inputs.quaternion[0] = wSignal.getValue();
    inputs.quaternion[1] = xSignal.getValue();
    inputs.quaternion[2] = ySignal.getValue();
    inputs.quaternion[3] = zSignal.getValue();

    if (inputs.accel == null) {
      inputs.accel = new double[3];
    }

    inputs.accel[0] = axSignal.getValue();
    inputs.accel[1] = aySignal.getValue();
    inputs.accel[2] = azSignal.getValue();
  }

  private Rotation3d acc = new Rotation3d();

  @Override
  public void SimPeriodic(Rotation3d delta) {
    acc = acc.plus(delta);
    imu.getSimState().setRoll(acc.getX() * UnitConversion.DEGREES_PER_RADIAN);
    imu.getSimState().setPitch(acc.getY() * UnitConversion.DEGREES_PER_RADIAN);
    imu.getSimState().setRawYaw(acc.getZ() * UnitConversion.DEGREES_PER_RADIAN);
  }
}
