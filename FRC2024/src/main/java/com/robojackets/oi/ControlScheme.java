package com.robojackets.oi;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class ControlScheme {

  public ChassisSpeeds getSwerveDriveInput() {
    return new ChassisSpeeds();
  }

  public boolean useRobotCentric() {
    return false;
  }

  public boolean resetYaw() {
    return false;
  }

  public boolean getLeftPaddle() {
    return false;
  }

  public boolean getRightPaddle() {
    return false;
  }

  public boolean toggleYawLock() {
    return false;
  }

  public String getName() {
    return "NONE";
  }

  public boolean sysidSteer() {
    return false;
  }

  public boolean sysidDriveTranslation() {
    return false;
  }

  public boolean sysidDriveRotation() {
    return false;
  }

  public boolean slipCurrent() {
    return false;
  }

  public boolean getCircleOperator() {
    return false;
  }

  public boolean getCircleDriver() {
    return false;
  }

  public boolean getSquareDriver() {
    return false;
  }

  public boolean getCrossOperator() {
    return false;
  }

  public boolean getCrossDriver() {
    return false;
  }

  // public double getIntakeCommand() {
  //   return 0.0;
  // }

  public boolean getTriangleOperator() {
    return false;
  }

  public boolean getTriangleDriver() {
    return false;
  }

  public boolean getSquareOperator() {
    return false;
  }

  public boolean getDPadUpOperator() {
    return false;
  }

  public boolean getDPadRightOperator() {
    return false;
  }

  public boolean getDPadDownOperator() {
    return false;
  }

  public boolean getDPadLeftOperator() {
    return false;
  }

  public boolean getDPadUpDriver() {
    return false;
  }

  public boolean getDPadRightDriver() {
    return false;
  }

  public boolean getDPadDownDriver() {
    return false;
  }

  public boolean getDPadLeftDriver() {
    return false;
  }

  public boolean getR1Driver() {
    return false;
  }

  public boolean getR1DriverReleased() {
    return false;
  }

  public boolean getL1Driver() {
    return false;
  }

  public boolean getR1Operator() {
    return false;
  }

  public boolean getL1Operator() {
    return false;
  }

  public double getClimberMove() {
    return 0.0;
  }

  public double getOptionsandLeftY() {
    return 0.0;
  }

  public double getOperatorLeftY() {
    return 0.0;
  }

  public boolean getOptionsPressedOperator() {
    return false;
  }

  public boolean getOptionsReleasedOperator() {
    return false;
  }

  public double getOperatorRightY() {
    return 0.0;
  }

  public boolean getTouchpadOperator() {
    return false;
  }

  public boolean getSharePressedOperator() {
    return false;
  }

  public boolean getShareReleasedOperator() {
    return false;
  }

  public boolean getShareOperator() {
    return false;
  }

  public boolean getPSOperator() {
    return false;
  }

  public boolean getPSPressedOperator() {
    return false;
  }

  public boolean getPSReleasedOperator() {
    return false;
  }

  public void setDriverRumble(double rumble) {}

  public double getR2Driver() {
    return 0.0;
  }

  public double getL2Driver() {
    return 0.0;
  }

  public double getR2Operator() {
    return 0.0;
  }

  public double getL2Operator() {
    return 0.0;
  }

  public boolean getShareButtonPressedDriver() {
    return false;
  }

  public boolean getL1ButtonTester() {
    return false;
  }
}
