package com.robojackets.auto;

import com.robojackets.Constants;
import com.robojackets.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turn180 extends Auton {
  private RobotContainer io;
  double targetYaw = 0.0;

  public Turn180(RobotContainer io) {
    this.io = io;
    targetYaw = io.swerve.ioInputs.getYawRadians() + Math.PI;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void Init() {
    // Must call
    super.Init();
    // SmartDashboard.putNumber("YawKp", 2.0);
  }

  @Override
  public void Periodic() {
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Current Yaw", io.swerve.ioInputs.getYawRadians());

    switch (getStep()) {
      case 0:
        // Code
        double yaw = io.swerve.ioInputs.getYawRadians();
        double yawError = targetYaw - yaw;
        double yawKP = SmartDashboard.getNumber("YawKp", 2.0);
        double yawLockCmd = (yawError * yawKP) + ((yawError / Math.abs(yawError)) * 0.3);
        double yawLockCmdMax = 0.7 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, yawLockCmd));
        if (Math.abs(yawError) < 0.0053) {
          Step();
        }

        break;

      case 1:
        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, 0));

        break;
    }
  }
}
