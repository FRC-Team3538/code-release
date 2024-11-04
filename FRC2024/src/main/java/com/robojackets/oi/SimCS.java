package com.robojackets.oi;

import com.robojackets.Constants;
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
  public ChassisSpeeds getSwerveDriveInput() {
    if (!driver.isConnected()) {
      return new ChassisSpeeds();
    }

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
    return driver.isConnected() && driver.getRawButton(1);
  }

  @Override
  public boolean resetYaw() {
    return driver.isConnected() && driver.getRawButton(2);
  }

  @Override
  public String getName() {
    return "Simulation";
  }

  @Override
  public boolean sysidSteer() {
    return driver.isConnected() && driver.getRawButton(3);
  }

  @Override
  public boolean sysidDriveTranslation() {
    return driver.isConnected() && driver.getRawButton(4);
  }

  @Override
  public boolean sysidDriveRotation() {
    return driver.isConnected() && driver.getRawButton(5);
  }
}
