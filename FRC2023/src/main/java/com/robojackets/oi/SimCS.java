package com.robojackets.oi;

import com.robojackets.Constants;
import com.robojackets.lib.ArcadeDriveInput;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;

public class SimCS extends ControlScheme {
  private final GenericHID driver;
  private final GenericHID operator;

  public SimCS(int driverID, int operatorID) {
    driver = new GenericHID(driverID);
    operator = new GenericHID(operatorID);
  }

  @Override
  public ChassisSpeeds getTankDriverInput() {
    double forward =
        -1 * driver.getRawAxis(1) * Constants.TankDrive.MaxLinearVelocityMetersPerSecond;
    double rotate =
        -1 * driver.getRawAxis(0) * Constants.TankDrive.MaxAngularVelocityRadiansPerSecond;

    return new ChassisSpeeds(forward, 0, rotate);
  }

  @Override
  public ArcadeDriveInput getTankArcadeInput() {
    double forwardVolts = -1 * driver.getRawAxis(1) * 12;
    double turnVolts = -1 * driver.getRawAxis(0) * 12;

    return new ArcadeDriveInput(forwardVolts, turnVolts);
  }

  @Override
  public ChassisSpeeds getSwerveDriverInput() {
    double forward =
        -1 * driver.getRawAxis(1) * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;
    double sideways =
        -1 * driver.getRawAxis(0) * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;
    double rotate =
        -1 * driver.getRawAxis(2) * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

    return new ChassisSpeeds(forward, sideways, rotate);
  }

  @Override
  public boolean useRobotCentric() {
    return driver.getRawButton(1);
  }

  @Override
  public boolean resetYaw() {
    return driver.getRawButton(2);
  }

  @Override
  public String getName() {
    return "Simulation";
  }
}
