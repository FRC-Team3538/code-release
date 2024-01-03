package com.robojackets.subsystems.tank;

import com.robojackets.subsystems.gearbox.GearboxIO;
import com.robojackets.subsystems.gearbox.GearboxIO.GearboxIOInputs;
import com.robojackets.subsystems.gyro.GyroIO;
import com.robojackets.subsystems.gyro.GyroIO.GyroIOInputs;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLog;

public class TankDriveIO {
  protected final GearboxIO leftGearbox;
  protected final GearboxIO rightGearbox;
  protected final GyroIO gyroIO;

  public TankDriveIO(GearboxIO leftGearbox, GearboxIO rightGearbox, GyroIO gyroIO) {
    this.leftGearbox = leftGearbox;
    this.rightGearbox = rightGearbox;
    this.gyroIO = gyroIO;

    setBrakeMode(false);
  }

  public void setBrakeMode(boolean brake) {
    leftGearbox.setBrakeMode(brake);
    rightGearbox.setBrakeMode(brake);
  }

  public void updateInputs(
      DriveIOInputs ioInputs,
      GearboxIOInputs leftIOInputs,
      GearboxIOInputs rightIOInputs,
      GyroIOInputs gyroIOInputs) {}

  public void setVoltage(DifferentialDriveWheelVoltages voltages) {}

  public void setWheelSpeeds(
      DifferentialDriveWheelSpeeds wheelSpeeds, DifferentialDriveWheelVoltages feedforward) {}

  @AutoLog
  @Getter
  public static class DriveIOInputs {
    double leftPositionMeters;
    double rightPositionMeters;

    double leftVelocityMetersPerSecond;
    double rightVelocityMetersPerSecond;

    boolean useFOC;
  }
}
