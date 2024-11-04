package com.robojackets.oi;

import com.robojackets.Constants;
import com.robojackets.lib.Deadband;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RealCS extends ControlScheme {

  private final PS4Controller driver;
  private final PS4Controller operator;
  private final PS4Controller tester;

  private final Deadband deadband;

  public RealCS(int driverID, int operatorID, int testerID) {
    driver = new PS4Controller(driverID);
    operator = new PS4Controller(operatorID);
    tester = new PS4Controller(testerID);

    deadband = new Deadband(0.1, 1);
  }

  @Override
  public ChassisSpeeds getSwerveDriveInput() {
    if (!driver.isConnected()) {
      return new ChassisSpeeds();
    }

    Vector<N2> leftStick = VecBuilder.fill(driver.getLeftY(), driver.getLeftX());
    leftStick = deadband.apply(leftStick, 2);

    double forward =
        -1 * leftStick.get(0, 0) * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;
    double sideways =
        -1 * leftStick.get(1, 0) * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;
    double rotate =
        -1
            * MathUtil.applyDeadband(driver.getRightX(), 0.1)
            * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

    // Taking out because we want trigger for sniper mode? I need to talk to Adam
    // about this....
    // if (driver.getR1Button()) {
    // rotate = rotate * 0.05;
    // forward = forward * 0.20;
    // sideways = sideways * 0.20;
    // }

    return new ChassisSpeeds(forward, sideways, rotate);
  }

  @Override
  public boolean getLeftPaddle() {
    return driver.isConnected() && driver.getL3Button();
  }

  @Override
  public boolean getRightPaddle() {
    return driver.isConnected() && driver.getR3Button();
  }

  @Override
  public boolean useRobotCentric() {
    return false; // driver.isConnected() && driver.getL1Button();
  }

  @Override
  public boolean resetYaw() {
    return driver.isConnected() && driver.getOptionsButtonPressed(); // Change to getPSButtonPressed
  }

  @Override
  public boolean toggleYawLock() {
    return driver.isConnected() && driver.getTouchpadPressed();
  }

  @Override
  public String getName() {
    return String.format("Dual Controller (Ports %d, %d)", driver.getPort(), operator.getPort());
  }

  @Override
  public boolean sysidSteer() {
    return driver.isConnected() && driver.getCircleButton();
  }

  @Override
  public boolean sysidDriveTranslation() {
    return false; // return driver.isConnected() && driver.getCrossButton();
  }

  @Override
  public boolean sysidDriveRotation() {
    return driver.isConnected() && driver.getSquareButton();
  }

  @Override
  public boolean slipCurrent() {
    return driver.isConnected() && driver.getTriangleButton();
  }

  @Override
  public boolean getCircleOperator() {
    return operator.isConnected() && operator.getCircleButton();
  }

  @Override
  public boolean getCrossOperator() {
    return operator.isConnected() && operator.getCrossButton();
  }

  @Override
  public boolean getCrossDriver() {
    return driver.isConnected() && driver.getCrossButton();
  }

  @Override
  public boolean getCircleDriver() {
    return driver.isConnected() && driver.getCircleButton();
  }

  @Override
  public boolean getSquareDriver() {
    return driver.isConnected() && driver.getSquareButton();
  }

  @Override
  public double getR2Driver() {
    if (driver.isConnected()) {
      return ((driver.getR2Axis() + 1.0) / 2.0); // Old One was 2
    } else {
      return 0.0;
    }
  }

  @Override
  public double getL2Driver() {
    if (driver.isConnected()) {
      return ((driver.getL2Axis() + 1.0) / 2.0); // Old One was 2
    } else {
      return 0.0;
    }
  }

  @Override
  public double getR2Operator() {
    if (operator.isConnected()) {
      return ((operator.getR2Axis() + 1.0) / 2.0); // Old One was 2
    } else {
      return 0.0;
    }
  }

  @Override
  public double getL2Operator() {
    if (operator.isConnected()) {
      return ((operator.getL2Axis() + 1.0) / 2.0); // Old One was 2
    } else {
      return 0.0;
    }
  }

  // @Override
  // public double getIntakeCommand() {
  // double dt = ((driver.getR2Axis() + 1.0) / 2.0) - ((driver.getL2Axis() + 1.0)
  // / 2.0);
  // double ot = ((operator.getR2Axis() + 1.0) / 2.0) - ((operator.getL2Axis() +
  // 1.0) / 2.0);

  // double motorvalue = MathUtil.applyDeadband(dt + ot, 0.05, 1);
  // SmartDashboard.putNumber("dt intake", dt);

  // return motorvalue;
  // }

  @Override
  public boolean getTriangleOperator() {
    return operator.isConnected() && operator.getTriangleButton();
  }

  @Override
  public boolean getTriangleDriver() {
    return driver.isConnected() && driver.getTriangleButton();
  }

  @Override
  public boolean getSquareOperator() {
    return operator.isConnected() && operator.getSquareButton();
  }

  @Override
  public boolean getDPadUpOperator() {
    return operator.isConnected() && operator.getPOV() == 0;
  }

  @Override
  public boolean getDPadRightOperator() {
    return operator.isConnected() && operator.getPOV() == 90;
  }

  @Override
  public boolean getDPadDownOperator() {
    return operator.isConnected() && operator.getPOV() == 180;
  }

  @Override
  public boolean getDPadLeftOperator() {
    return operator.isConnected() && operator.getPOV() == 270;
  }

  @Override
  public boolean getDPadUpDriver() {
    return driver.isConnected() && driver.getPOV() == 0;
  }

  @Override
  public boolean getDPadRightDriver() {
    return driver.isConnected() && driver.getPOV() == 90;
  }

  @Override
  public boolean getDPadDownDriver() {
    return driver.isConnected() && driver.getPOV() == 180;
  }

  @Override
  public boolean getDPadLeftDriver() {
    return driver.isConnected() && driver.getPOV() == 270;
  }

  @Override
  public boolean getL1Operator() {
    return (operator.isConnected() && operator.getL1Button());
  }

  @Override
  public boolean getR1Operator() {
    return (operator.isConnected() && operator.getR1Button());
  }

  @Override
  public boolean getR1Driver() {
    return (driver.isConnected() && driver.getR1Button());
  }

  @Override
  public boolean getR1DriverReleased() {
    return (driver.isConnected() && driver.getR1ButtonReleased());
  }

  @Override
  public boolean getL1Driver() {
    return (driver.isConnected() && driver.getL1Button());
  }

  @Override
  public double getOperatorLeftY() {
    if (!operator.getOptionsButton()) {
      return MathUtil.applyDeadband(operator.getLeftY(), 0.05);
    } else {
      return 0.0;
    }
  }

  @Override
  public double getOptionsandLeftY() {
    if (operator.getOptionsButton()) {
      return MathUtil.applyDeadband(operator.getLeftY(), 0.08, 1);
    } else {
      return 0.0;
    }
  }

  @Override
  public boolean getOptionsPressedOperator() {
    return operator.getOptionsButtonPressed();
  }

  @Override
  public boolean getOptionsReleasedOperator() {
    return operator.getOptionsButtonReleased();
  }

  @Override
  public boolean getSharePressedOperator() {
    return operator.getShareButtonPressed();
  }

  @Override
  public boolean getShareReleasedOperator() {
    return operator.getShareButtonReleased();
  }

  @Override
  public double getOperatorRightY() {
    return MathUtil.applyDeadband(operator.getRightY(), 0.05);
  }

  @Override
  public boolean getTouchpadOperator() {
    return operator.getTouchpad();
  }

  @Override
  public boolean getShareOperator() {
    return operator.getShareButton();
  }

  @Override
  public boolean getPSOperator() {
    return operator.getPSButton();
  }

  @Override
  public boolean getPSPressedOperator() {
    return operator.getPSButtonPressed();
  }

  @Override
  public boolean getPSReleasedOperator() {
    return operator.getPSButtonReleased();
  }

  @Override
  public boolean getShareButtonPressedDriver() {
    return driver.getShareButtonPressed();
  }

  @Override
  public boolean getL1ButtonTester() {
    return tester.getL1Button();
  }

  @Override
  public void setDriverRumble(double rumble) {
    driver.setRumble(GenericHID.RumbleType.kBothRumble, rumble);
    operator.setRumble(GenericHID.RumbleType.kBothRumble, rumble);
    SmartDashboard.putNumber("rumbler", rumble);
  }
}
