package com.robojackets.auto;

import com.robojackets.Constants;
import com.robojackets.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;

public class BiggestBoi extends Auton {
  private RobotContainer io;
  double targetYaw = 0.0;

  public BiggestBoi(RobotContainer io) {
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
        double yawKP = 1.0;
        double yawLockCmd = yawError * yawKP;
        double yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.6, -0.06, yawLockCmd));

        // boolean RobotOnDescent = Math.abs(io.swerve.ioInputs.getPitchRadians() *
        // 180.0 /
        // Math.PI) < 15.0;

        if (getStepElapsed() > 2.65) {
          Step();
        }

        // case 5:
        // boolean RobotOnDescent = Math.abs(io.swerve.ioInputs.getPitchRadians() *
        // 180.0 /
        // Math.PI) < 15.0;

        break;

      case 5:
        io.swerve.DriveRobotCentric(new ChassisSpeeds(0, 0, 0));
        if (getStepElapsed() > 0.2) {
          targetYaw = targetYaw + Math.PI;
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
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 7:
        io.intakemotor.set(-0.15);
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
        if (getStepElapsed() > 1.2) {
          Step();
          targetYaw = targetYaw - Math.PI;
        }

        break;

      case 8:
        io.intakemotor.set(-0.15);
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
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 9:
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

        io.swerve.DriveRobotCentric(new ChassisSpeeds(1.6, 0.085, yawLockCmd));

        boolean robotIsCimbing1 =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 6.5;

        // boolean RobotOnDescent = Math.abs(io.swerve.ioInputs.getPitchRadians() *
        // 180.0 /
        // Math.PI) < 15.0;

        if (getStepElapsed() > 4.4 || robotIsCimbing1) {
          Step();
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 10:
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.03;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.15, 0, yawLockCmd)); // 2.6 next change?
        if (getStepElapsed() > 0.65) {
          Step();
          io.arm.GoTo(1.0, 132.0);
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 11:

        //
        // Yaw Lock
        //
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.03;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        //
        // Strafe Lock
        //
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry tid = table.getEntry("tid");

        double strafeError = tx.getDouble(0.0);
        double strafeKP = -0.10;
        double strafeCmd = strafeError * strafeKP;
        double strafeCmdMax = 0.15 * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;

        // Limit the yawLockCmd
        if (Math.abs(strafeError) < 2.0) {
          strafeCmd = 0.0;
        } else if (strafeCmd > strafeCmdMax) {
          strafeCmd = strafeCmdMax;
        } else if (strafeCmd < -strafeCmdMax) {
          strafeCmd = -strafeCmdMax;
        }

        //
        // Forward Lock
        //

        NetworkTableEntry ta = table.getEntry("ta");

        double tavalue = ta.getDouble(0.0);

        double targetFwd = 1.6;
        double fwdError = targetFwd - tavalue;
        double fwdKP = 0.55;
        double fwdLockCmd = fwdError * fwdKP;
        double fwdLockCmdMax = 0.15 * Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;

        // Limit the fwdLockCmd
        if (Math.abs(fwdError) < 0.1) {
          fwdLockCmd = 0.0;
        }
        if (fwdLockCmd > fwdLockCmdMax) {
          fwdLockCmd = fwdLockCmdMax;
        } else if (fwdLockCmd < -fwdLockCmdMax) {
          fwdLockCmd = -fwdLockCmdMax;
        }

        // Driving Robot
        io.swerve.DriveRobotCentric(new ChassisSpeeds(fwdLockCmd, strafeCmd, yawLockCmd));

        if (tavalue > 1.50 && Math.abs(strafeError) < 3.0) {
          Step();
        }

        break;

      case 12:
        io.intakemotor.set(-0.15);
        io.swerve.DriveRobotCentric(new ChassisSpeeds(0.5, 0.0, 0.0));
        if (getStepElapsed() > 1.0) {
          Step();
        }
        break;

      case 13:
        io.intakemotor.set(-0.15);
        if (io.arm.GoTo(16.3, 229.0)) {
          Step();
        }
        break;

      case 14:
        io.intakemotor.set(0.1);
        if (getStepElapsed() > 1.0) {
          Step();
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }
        break;

      case 15:
        io.arm.GoTo(1.0, 90.0);
        io.intakemotor.set(0.0);

        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.03;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // Limit the yawLockCmd
        if (yawLockCmd > yawLockCmdMax) {
          yawLockCmd = yawLockCmdMax;
        } else if (yawLockCmd < -yawLockCmdMax) {
          yawLockCmd = -yawLockCmdMax;
        }

        boolean robotIsCimbing2 =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 6.5;

        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.6, -0.2, yawLockCmd));
        if (getStepElapsed() > 4.7 || robotIsCimbing2) {
          Step();
          targetYaw = io.swerve.ioInputs.getYawRadians();
        }

        break;

      case 16:
        yaw = io.swerve.ioInputs.getYawRadians();
        yawError = targetYaw - yaw;
        yawKP = 1.03;
        yawLockCmd = yawError * yawKP;
        yawLockCmdMax = 0.5 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.15, 0, yawLockCmd)); // 2.6 next change?
        if (getStepElapsed() > 0.65) {
          Step();
        }

        break;

      case 17:
        io.intakemotor.set(-0.0);
        double pitch = -io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI;
        double kP = 0.020;
        double vx = pitch * kP;

        boolean robotIsLevel =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 2.0;
        if (!robotIsLevel) {
          io.swerve.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
        } else {
          io.swerve.Cross();
        }
        break;

        // case 8:
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        // io.arm.GoTo(1.0, 90.0);
        // io.intakemotor.set(-0.15);
        // if (getStepElapsed() > 0.3) {
        // // targetYaw = targetYaw - Math.PI;
        // Step();
        // }
        // break;

        // case 9:
        // yaw = io.swerve.ioInputs.getYawRadians();
        // yawError = targetYaw - yaw;
        // yawKP = 2.0;
        // yawLockCmd = yawError * yawKP;
        // yawLockCmdMax = 0.7 *
        // Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // // Limit the yawLockCmd
        // if (yawLockCmd > yawLockCmdMax) {
        // yawLockCmd = yawLockCmdMax;
        // } else if (yawLockCmd < -yawLockCmdMax) {
        // yawLockCmd = -yawLockCmdMax;
        // }
        // io.intakemotor.set(-0.15);
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(0.0, 0.0, yawLockCmd));

        // if (Math.abs(yawError) < 0.1) {
        // Step();
        // }

        // break;

        // OLD STATE FOR DRIVING BACK
        // case 9:
        // io.arm.GoTo(1.0, 90.0);
        // io.intakemotor.set(-0.15);
        // targetYaw = io.swerve.ioInputs.getYawRadians();
        // yaw = io.swerve.ioInputs.getYawRadians();
        // yawError = targetYaw - yaw;
        // yawKP = 1.03;
        // yawLockCmd = yawError * yawKP;
        // yawLockCmdMax = 0.5 *
        // Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // // Limit the yawLockCmd
        // if (yawLockCmd > yawLockCmdMax) {
        // yawLockCmd = yawLockCmdMax;
        // } else if (yawLockCmd < -yawLockCmdMax) {
        // yawLockCmd = -yawLockCmdMax;
        // }

        // boolean robotIsCimbing =
        // Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 6.5;

        // io.swerve.DriveRobotCentric(new ChassisSpeeds(-2.2, -0.2, yawLockCmd));
        // if (getStepElapsed() > 4.7 || robotIsCimbing) {
        // Step();
        // }

        // break;

        // case 10:
        // targetYaw = io.swerve.ioInputs.getYawRadians();
        // yaw = io.swerve.ioInputs.getYawRadians();
        // yawError = targetYaw - yaw;
        // yawKP = 1.03;
        // yawLockCmd = yawError * yawKP;
        // yawLockCmdMax = 0.5 *
        // Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.15, 0, yawLockCmd)); // 2.6
        // next change?
        // if (getStepElapsed() > 0.65) {
        // Step();
        // }

        // break;

        // case 11:
        // io.intakemotor.set(-0.1);
        // double pitch = -io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI;
        // double kP = 0.020;
        // double vx = pitch * kP;

        // boolean robotIsLevel =
        // Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 2.0;
        // if (!robotIsLevel) {
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
        // } else {
        // io.swerve.Cross();
        // }
        // break;
        // }
        // case 0:
        // // Move arm out of camera view
        // io.arm.GoTo(1, 132.0);
        // io.intakemotor.set(-0.1);

        // //
        // // Yaw Lock
        // //
        // double yaw = io.swerve.ioInputs.getYawRadians();
        // double yawError = targetYaw - yaw;
        // double yawKP = 0.3;
        // double yawLockCmd = yawError * yawKP;
        // double yawLockCmdMax = 0.2 *
        // Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

        // // Limit the yawLockCmd
        // if (yawLockCmd > yawLockCmdMax) {
        // yawLockCmd = yawLockCmdMax;
        // } else if (yawLockCmd < -yawLockCmdMax) {
        // yawLockCmd = -yawLockCmdMax;
        // }

        // //
        // // Strafe Lock
        // //
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry tid = table.getEntry("tid");

        // double strafeError = tx.getDouble(0.0);
        // double strafeKP = -0.10;
        // double strafeCmd = strafeError * strafeKP;
        // double strafeCmdMax = 0.15 *
        // Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;

        // // Limit the yawLockCmd
        // if (Math.abs(strafeError) < 2.0) {
        // strafeCmd = 0.0;
        // } else if (strafeCmd > strafeCmdMax) {
        // strafeCmd = strafeCmdMax;
        // } else if (strafeCmd < -strafeCmdMax) {
        // strafeCmd = -strafeCmdMax;
        // }

        // //
        // // Forward Lock
        // //

        // NetworkTableEntry ta = table.getEntry("ta");

        // double tavalue = ta.getDouble(0.0);

        // double targetFwd = 1.6;
        // double fwdError = targetFwd - tavalue;
        // double fwdKP = 0.55;
        // double fwdLockCmd = fwdError * fwdKP;
        // double fwdLockCmdMax = 0.15 *
        // Constants.SwerveDrive.MaxLinearVelocityMetersPerSecond;

        // // Limit the fwdLockCmd
        // if (Math.abs(fwdError) < 0.1) {
        // fwdLockCmd = 0.0;
        // }
        // if (fwdLockCmd > fwdLockCmdMax) {
        // fwdLockCmd = fwdLockCmdMax;
        // } else if (fwdLockCmd < -fwdLockCmdMax) {
        // fwdLockCmd = -fwdLockCmdMax;
        // }

        // // Driving Robot
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(fwdLockCmd, strafeCmd,
        // yawLockCmd));

        // if (tavalue > 1.50 && Math.abs(strafeError) < 3.0) {
        // Step();
        // }

        // break;

        // case 1:
        // io.intakemotor.set(-0.1);
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(0.5, 0.0, 0.0));
        // if (getStepElapsed() > 1.0) {
        // Step();
        // }
        // break;

        // case 2:
        // io.intakemotor.set(-0.1);
        // if (io.arm.GoTo(16.3, 229.0)) {
        // Step();
        // }
        // break;

        // case 3:
        // io.intakemotor.set(0.3);
        // if (getStepElapsed() > 1.0) {
        // Step();
        // }
        // break;
        // case 4:
        // io.arm.GoTo(1.0, 90.0);
        // io.swerve.DriveRobotCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        // }
    }
  }
}
