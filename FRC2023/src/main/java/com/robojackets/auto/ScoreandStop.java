package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.subsystems.ArmSetpoint;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ScoreandStop extends Auton {
  private RobotContainer io;

  public ScoreandStop(RobotContainer io) {
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
        io.swerve.Cross();
        break;
    }
  }
}
