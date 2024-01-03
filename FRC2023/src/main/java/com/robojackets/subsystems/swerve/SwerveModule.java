package com.robojackets.subsystems.swerve;

import com.robojackets.subsystems.gearbox.GearboxIOInputsAutoLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.AllArgsConstructor;
import org.littletonrobotics.junction.Logger;

@AllArgsConstructor
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged ioInputs = new SwerveModuleIOInputsAutoLogged();
  private final GearboxIOInputsAutoLogged driveInputs = new GearboxIOInputsAutoLogged();
  private final GearboxIOInputsAutoLogged azimuthInputs = new GearboxIOInputsAutoLogged();

  private final Translation2d location;

  private final String name;

  public Translation2d getLocation() {
    return location;
  }

  public void periodic() {
    this.io.updateInputs(ioInputs, driveInputs, azimuthInputs);
    Logger.getInstance().processInputs(String.format("Swerve/%s", name), ioInputs);
    Logger.getInstance().processInputs(String.format("Swerve/%s/Drive", name), driveInputs);
    Logger.getInstance().processInputs(String.format("Swerve/%s/Azimuth", name), azimuthInputs);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        ioInputs.velocityMetersPerSecond, Rotation2d.fromRadians(ioInputs.angleRadians));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        ioInputs.distanceMeters, Rotation2d.fromRadians(ioInputs.angleRadians));
  }

  public void Drive(SwerveModuleState targetState) {

    Logger.getInstance().recordOutput(String.format("Swerve/%s/targetState", name), targetState);

    var state =
        SwerveModuleState.optimize(targetState, Rotation2d.fromRadians(ioInputs.angleRadians));

    Logger.getInstance().recordOutput(String.format("Swerve/%s/optimizedState", name), state);

    io.setDesiredState(state); // setDesiredState coming from FalconSwerveModuleIO?
  }

  public void Stop() {
    io.Stop();
  }

  public void simPeriodic() {
    io.SimPeriodic();
  }
}
