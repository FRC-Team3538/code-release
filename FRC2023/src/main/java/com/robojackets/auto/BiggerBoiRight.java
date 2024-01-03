package com.robojackets.auto;

import com.robojackets.Constants;
import com.robojackets.RobotContainer;
import com.robojackets.subsystems.ArmSetpoint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BiggerBoiRight extends Auton {
  private RobotContainer io;
  double targetYaw = 0.0;

  public BiggerBoiRight(RobotContainer io) {
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

    // TODO: Change GoTo Functions from just inches and degrees to SetPoints

    SmartDashboard.putNumber("state machine case", getStep());
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Current Yaw", io.swerve.ioInputs.getYawRadians());

    switch (getStep()) {
      case 0:
        io.intakemotor.set(0.1);

        if (io.arm.GoTo(1.0, 90.0)) {
          Step();
        }
        break;

      case 1:
        io.intakemotor.set(0.1);

        if (io.arm.GoTo(16.3, 229.0)) {
          Step();
        }

        break;

      case 2:
        io.intakemotor.set(-0.3);

        if (getStepElapsed() > 0.2) {
          Step();
        }
        break;

      case 3:
        io.intakemotor.set(0.0);
        if (io.arm.GoTo(1.0, 90.0)) {
          Step();
        }
        break;

      case 4:

        // Yaw lock
        //
        double yaw = io.swerve.ioInputs.getYawRadians();
        double yawError = targetYaw - yaw;
        double yawKP = 2.0;
        double yawLockCmd = yawError * yawKP;
        double yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.7, -0.1, yawLockCmd));

        //   boolean RobotOnDescent = Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 /
        // Math.PI) < 15.0;

        if (getStepElapsed() > 2.72) {
          Step();
        }

        // case 5:
        //   boolean RobotOnDescent =  Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 /
        // Math.PI) < 15.0;

        break;

      case 5:
        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, 0));
        if (getStepElapsed() > 0.2) {
          targetYaw = targetYaw - 3.05433;
          Step();
        }
        break;

      case 6:
        // Code
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 2.0;
        yawLockCmd = (yawError * yawKP) + ((yawError / Math.abs(yawError)) * 0.3);
        yawLockCmdMax = 0.7 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, yawLockCmd));
        io.arm.GoTo(0.86, 4.85);
        if (Math.abs(yawError) < 0.0053) {
          Step();
          // targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 7:
        io.intakemotor.set(-0.25);
        io.arm.GoTo(0.86, 4.85);
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.0;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(0.72, 0.0, yawLockCmd));
        if (getStepElapsed() > 0.95) {
          targetYaw = targetYaw + 3.05433;
          Step();
        }

        break;

        // case 8:
        //   io.swerve.DriveRobotCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        //   io.arm.GoTo(1.0, 90.0);
        //   io.intakemotor.set(-0.15);
        //   if (getStepElapsed() > 0.3) {
        //     // targetYaw = targetYaw - Math.PI;
        //     Step();
        //   }
        //   break;

        // case 9:
        //   yaw = io.swerve.ioInputs.getYawRadians();
        //   yawError = targetYaw - yaw;
        //   yawKP = 2.0;
        //   yawLockCmd = yawError * yawKP;
        //   yawLockCmdMax = 0.7 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        //   // Limit the yawLockCmd
        //   if (yawLockCmd > yawLockCmdMax) {
        //     yawLockCmd = yawLockCmdMax;
        //   } else if (yawLockCmd < -yawLockCmdMax) {
        //     yawLockCmd = -yawLockCmdMax;
        //   }
        //   io.intakemotor.set(-0.15);
        //   io.swerve.DriveRobotCentric(new ChassisSpeeds(0.0, 0.0, yawLockCmd));

        //   if (Math.abs(yawError) < 0.1) {
        //     Step();
        //   }

        //   break;

      case 8:
        io.intakemotor.set(-0.25);
        io.arm.GoTo(1.0, 90.0);
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 2.0;
        yawLockCmd = (yawError * yawKP) + ((yawError / Math.abs(yawError)) * 0.3);
        yawLockCmdMax = 0.7 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, yawLockCmd));
        if (Math.abs(yawError) < 0.0053) {
          Step();
          // targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 9:
        io.arm.GoTo(1.0, 90.0);
        io.intakemotor.set(-0.15);
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 3.0;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.55 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        boolean robotIsCimbing =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 6.5;

        io.swerve.DriveRobotCentric(new ChassisSpeeds(2.2, -0.3, yawLockCmd));
        if (getStepElapsed() > 4.7 || robotIsCimbing) {
          Step();
          // targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 10:
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.03;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        io.swerve.DriveRobotCentric(new ChassisSpeeds(1.5, 0, yawLockCmd)); // 2.6 next change?
        if (getStepElapsed() > 0.85) {
          Step();
        }

        break;

      case 11:
        io.intakemotor.set(-0.15);
        double pitch = -io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI;
        double kP = 0.020;
        double vx = pitch * kP;

        boolean robotIsLevel =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 2.0;
        if (!robotIsLevel) {
          io.swerve.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
        } else {
          io.swerve.Cross();
          io.arm.GoTo(ArmSetpoint.LOW_SCORE);
        }
        if (io.arm.getElbowPosition() < 47.5) {
          io.intakemotor.set(1.0);
        }

        break;
    }
  }
}
