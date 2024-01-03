package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.subsystems.ArmSetpoint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;

public class BigBoi extends Auton {
  private RobotContainer io;
  double targetYaw = 0.0;

  public BigBoi(RobotContainer io) {
    this.io = io;
    targetYaw = io.swerve.ioInputs.getYawRadians();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void Init() {
    // Must call
    super.Init();
  }

  @Override
  public void Periodic() {

    switch (getStep()) {
      case 0:
        io.intakemotor.set(0.1);

        if (io.arm.GoTo(ArmSetpoint.STOW)) {
          Step();
        }
        break;

      case 1:
        io.intakemotor.set(0.1);

        if (io.arm.GoTo(ArmSetpoint.BIGBOI_HIGH)) {
          Step();
        }

        break;

      case 2:
        io.intakemotor.set(-0.5);

        if (getStepElapsed() > 1.0) {
          Step();
        }
        break;
      case 3:
        io.intakemotor.set(0.0);
        if (io.arm.GoTo(ArmSetpoint.STOW)) {
          Step();
        }
        break;
      case 4:
        // double yaw = io.swerve.ioInputs.getYawRadians();
        // double yawError = targetYaw - yaw;
        // double yawKP = 1.0;
        // double yawLockCmd = yawError * yawKP;
        // double yawLockCmdMax = 0.2 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        // if (yawLockCmd > yawLockCmdMax) {
        //   yawLockCmd = yawLockCmdMax;
        // } else if (yawLockCmd < -yawLockCmdMax) {
        //   yawLockCmd = -yawLockCmdMax;
        // }
        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.5, 0.0, 0.0));

        // boolean robotIsCimbing =
        //     Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 15.0;
        if (getStepElapsed() > 3.15) {
          Step();
        }

        break;

      case 5:
        // Code
        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, 0));

        if (getStepElapsed() > 0.25) {
          Step();
        }

        break;

      case 6:
        // double yaw2 = io.swerve.ioInputs.getYawRadians();
        // double yawError2 = targetYaw - yaw2;
        // double yawKP2 = 1.0;
        // double yawLockCmd2 = yawError2 * yawKP2;
        // double yawLockCmdMax2 = 0.2 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        // if (yawLockCmd2 > yawLockCmdMax2) {
        //   yawLockCmd2 = yawLockCmdMax2;
        // } else if (yawLockCmd2 < -yawLockCmdMax2) {
        //   yawLockCmd2 = -yawLockCmdMax2;
        // }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(2.1, 0, 0.0));

        boolean robotIsCimbing =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 15.0;
        if (getStepElapsed() > 3.5 || robotIsCimbing) {
          Step();
        }

        break;

      case 7:
        double pitch = -io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI;
        double kP = 0.017;
        double vx = pitch * kP;

        boolean robotIsLevel =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 6.0;
        if (!robotIsLevel) {
          io.swerve.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
        } else {
          io.swerve.Cross();
        }
        break;
    }
  }
}
