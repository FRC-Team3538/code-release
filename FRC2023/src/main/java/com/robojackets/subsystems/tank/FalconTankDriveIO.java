package com.robojackets.subsystems.tank;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.robojackets.subsystems.gearbox.FalconGearboxIO;
import com.robojackets.subsystems.gearbox.GearboxIO.GearboxIOInputs;
import com.robojackets.subsystems.gyro.Pigeon2GyroIO;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.stream.Stream;

public class FalconTankDriveIO extends TankDriveIO {

  private static final double kWheelRadiusMeters = Units.inchesToMeters(1.95);
  private static final double kGearRatio = (60.0 / 11.0) * (54.0 / 38.0);
  private static final double kMetersPerRotation = (2 * kWheelRadiusMeters * Math.PI) / kGearRatio;

  private final BooleanEntry useFOCEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("/TankIO/useFOC").getEntry(false);

  public FalconTankDriveIO() {
    super(
        new FalconGearboxIO(new TalonFX(0), new TalonFX(1), new TalonFX(2)),
        new FalconGearboxIO(new TalonFX(3), new TalonFX(4), new TalonFX(5)),
        new Pigeon2GyroIO(new Pigeon2(30)));

    super.leftGearbox.setCurrentLimit(40, true);
    super.rightGearbox.setCurrentLimit(40, true);

    leftGearbox.setInverted(false);
    rightGearbox.setInverted(true);

    setBrakeMode(false);

    useFOCEntry.set(false);

    if (leftGearbox.isOnCanivore() && rightGearbox.isOnCanivore() && gyroIO.isOnCanivore()) {
      var leftSignals = ((FalconGearboxIO) leftGearbox).getCriticalSignals();
      var rightSignals = ((FalconGearboxIO) rightGearbox).getCriticalSignals();
      var imuSignals = ((Pigeon2GyroIO) gyroIO).getCriticalSignals();

      var signals =
          Stream.concat(
                  Stream.concat(leftSignals.stream(), rightSignals.stream()), imuSignals.stream())
              .toArray(size -> new StatusSignalValue[size]);

      BaseStatusSignalValue.waitForAll(0.02, signals);
    }
  }

  @Override
  public void setBrakeMode(boolean brake) {
    leftGearbox.setBrakeMode(brake);
    rightGearbox.setBrakeMode(brake);
  }

  public void updateInputs(
      DriveIOInputs ioInputs, GearboxIOInputs leftIOInputs, GearboxIOInputs rightIOInputs) {
    leftGearbox.updateInputs(leftIOInputs);
    rightGearbox.updateInputs(rightIOInputs);

    ioInputs.leftPositionMeters = leftIOInputs.getPosition() * kMetersPerRotation;
    ioInputs.leftVelocityMetersPerSecond = leftIOInputs.getVelocity() * kMetersPerRotation;
    ioInputs.rightPositionMeters = rightIOInputs.getPosition() * kMetersPerRotation;
    ioInputs.rightVelocityMetersPerSecond = rightIOInputs.getVelocity() * kMetersPerRotation;

    ioInputs.useFOC = useFOCEntry.get();
  }

  @Override
  public void setVoltage(DifferentialDriveWheelVoltages voltages) {
    leftGearbox.setControl(new VoltageOut(voltages.left, useFOCEntry.get(), false));
    rightGearbox.setControl(new VoltageOut(voltages.right, useFOCEntry.get(), false));
  }

  @Override
  public void setWheelSpeeds(
      DifferentialDriveWheelSpeeds wheelSpeeds, DifferentialDriveWheelVoltages wheelVoltages) {
    leftGearbox.setControl(
        new VelocityVoltage(kGearRatio, useFOCEntry.get(), wheelVoltages.left, 0, false));
    rightGearbox.setControl(
        new VelocityVoltage(kGearRatio, useFOCEntry.get(), wheelVoltages.right, 0, false));
  }
}
