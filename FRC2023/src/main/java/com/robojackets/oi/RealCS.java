package com.robojackets.oi;

import com.robojackets.Constants;
import com.robojackets.lib.ArcadeDriveInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RealCS extends ControlScheme {

  private final PS4Controller driver;
  private final PS4Controller operator;

  public RealCS(int driverID, int operatorID) {
    driver = new PS4Controller(driverID);
    operator = new PS4Controller(operatorID);
  }

  @Override
  public ChassisSpeeds getTankDriverInput() {
    double forward =
        MathUtil.applyDeadband(
            -1 * driver.getLeftX(), 0.1, Constants.TankDrive.MaxLinearVelocityMetersPerSecond);
    double rotate =
        MathUtil.applyDeadband(
            -1 * driver.getRightY(), 0.1, Constants.TankDrive.MaxAngularVelocityRadiansPerSecond);

    return new ChassisSpeeds(forward, 0, rotate);
  }

  @Override
  public ArcadeDriveInput getTankArcadeInput() {
    double forwardVolts = -1 * MathUtil.applyDeadband(driver.getLeftY(), 0.1, 12);
    double turnVolts = -1 * MathUtil.applyDeadband(driver.getRightX(), 0.1, 12);
    return new ArcadeDriveInput(forwardVolts, turnVolts);
  }

  @Override
  public ChassisSpeeds getSwerveDriverInput() {
    double correctionfactor;
    double correctionangle;
    double magnitude;
    double leftY = driver.getLeftY();
    double leftX = driver.getLeftX();
    double rightX = driver.getRightX();
    double forward;
    double sideways;
    double rotate;

    magnitude = Math.sqrt((leftY * leftY) + (leftX * leftX));
    correctionangle = (Math.atan2(leftX, leftY) - (Math.PI / 2));
    correctionfactor = (rightX * -0.4 * magnitude);
    if ((Math.abs(leftY) > 0.1) || (Math.abs(leftX) > 0.1) || (Math.abs(rightX) > 0.1)) {
      forward =
          (-1
              * (MathUtil.applyDeadband(leftY, 0.1)
                  + (correctionfactor * Math.cos(correctionangle)))
              * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond);

      sideways =
          (-1
              * (MathUtil.applyDeadband(leftX, 0.1)
                  + (correctionfactor * Math.sin(correctionangle)))
              * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond);

      rotate =
          -1
              * MathUtil.applyDeadband(driver.getRightX(), 0.1)
              * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
    } else {
      forward = 0;
      sideways = 0;
      rotate = 0;
    }
    if (driver.getR1Button()) {
      rotate = rotate * 0.05;
      forward = forward * 0.20;
      sideways = sideways * 0.20;
    }

    SmartDashboard.putNumber("driver.getLeftY()", driver.getLeftY());
    SmartDashboard.putNumber("CorrectionMagnitude", correctionfactor);
    SmartDashboard.putNumber("CorrectionAngle", correctionangle);
    SmartDashboard.putNumber("driver.getLeftX()", driver.getLeftX());
    SmartDashboard.putNumber("driver.getRightX()", driver.getRightX());
    return new ChassisSpeeds(forward, sideways, rotate);
  }

  @Override
  public boolean useRobotCentric() {
    return driver.getL1Button();
  }

  @Override
  public boolean resetYaw() {
    return driver.getOptionsButtonPressed();
  }

  @Override
  public boolean toggleYawLock() {
    return driver.getTouchpadPressed();
  }

  @Override
  public boolean sensorOverrideEnable() {
    return operator.getPSButtonPressed();
  }

  @Override
  public boolean sensorOverrideDisable() {
    return operator.getPSButtonReleased();
  }

  // @Override
  // public boolean SetFieldCentric() {
  // return driver.getR1ButtonPressed();
  // }

  // @Override
  // public boolean SetRobotCentric() {
  // return driver.getL1ButtonPressed();
  // }

  @Override
  public boolean SliderSetZero() {
    return operator.getShareButtonPressed();
  }

  @Override
  public boolean ElbowSetZero() {
    return operator.getOptionsButtonPressed();
  }

  @Override
  public double getIntakeCommand() {
    double motorvalue =
        MathUtil.applyDeadband(
                driver.getR2Axis() * driver.getR2Axis() * driver.getR2Axis()
                    - driver.getL2Axis()
                    + operator.getR2Axis() * operator.getR2Axis() * operator.getR2Axis()
                    - operator.getL2Axis(),
                0.05,
                1)
            * (1.0);
    SmartDashboard.putNumber("TriggerValue", motorvalue);
    return motorvalue;
  }

  @Override
  public double getSliderCommand() {
    double joystickvalue = -1 * operator.getLeftY();
    double slidervalue = MathUtil.applyDeadband(joystickvalue, 0.07, 1);
    SmartDashboard.putNumber("SliderValue", slidervalue);
    SmartDashboard.putNumber("rawslider", joystickvalue);
    return slidervalue;
  }

  @Override
  public double getElbowCommand() {
    double elbowvalue = MathUtil.applyDeadband(-1 * operator.getRightY(), 0.07, 1) * (0.3);
    SmartDashboard.putNumber("ElbowValue", elbowvalue);
    return elbowvalue;
  }

  @Override
  public boolean DecElbowCommand() {
    return operator.getL1ButtonPressed();
  }

  @Override
  public boolean IncElbowCommand() {
    return operator.getR1ButtonPressed();
  }

  @Override
  public boolean getPresetShelf() {
    return (driver.getPOV() == 0) || (operator.getPOV() == 0);
  }

  @Override
  public boolean getPresetHighMilford() {
    return driver.getTriangleButton() || (operator.getTriangleButton());
  }

  @Override
  public boolean getPresetGround() {
    return (driver.getPOV() == 180) || (operator.getPOV() == 180);
  }

  @Override
  public boolean getPresetStow() {
    return (driver.getPOV() == 90) || (operator.getPOV() == 90);
  }

  @Override
  public boolean getPresetMid() {
    return (driver.getSquareButton() || operator.getSquareButton());
  }

  @Override
  public boolean getPresetHighMuskegon() {
    return (driver.getCircleButton() || operator.getCircleButton());
  }

  @Override
  public boolean getPresetLowScore() {
    return operator.getCrossButton() || driver.getCrossButton();
  }

  @Override
  public String getName() {
    return "Dual Controller (Ports 0, 1)";
  }
}
