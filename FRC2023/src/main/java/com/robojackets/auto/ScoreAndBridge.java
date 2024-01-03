package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.subsystems.ArmSetpoint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ScoreAndBridge extends Auton {
  private RobotContainer io;

  public ScoreAndBridge(RobotContainer io) {
    this.io = io;
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
        io.intakemotor.set(0.3);

        if (io.arm.GoTo(ArmSetpoint.STOW)) {
          Step();
        }
        break;

      case 1:
        io.intakemotor.set(0.3);

        if (io.arm.GoTo(ArmSetpoint.BIGBOI_HIGH)) {
          Step();
        }

        break;

      case 2:
        io.intakemotor.set(-0.5);

        if (getStepElapsed() > 2.0) {
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
        io.swerve.DriveRobotCentric(new ChassisSpeeds(-1.5, 0.0, 0.0));

        boolean robotIsCimbing =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) > 15.0;
        if (getStepElapsed() > 2.5 || robotIsCimbing) {
          Step();
        }

        break;
      case 5:
        double pitch = -io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI;
        double kP = 0.017;
        double vx = pitch * kP;

        boolean robotIsLevel =
            Math.abs(io.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 2.0;
        if (!robotIsLevel) {
          io.swerve.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
        } else {
          io.swerve.Cross();
        }
        break;
    }
  }
}
