package com.robojackets.auto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public abstract class Auton implements Sendable {
  private Timer autonTimer = new Timer();
  private int step = 0;
  private Timer stepTimer = new Timer();

  public void Init() {
    autonTimer.reset();
    autonTimer.start();

    step = 0; // For safety purposes (it does auto again after disable and enable)
    stepTimer.reset();
    stepTimer.start();
  }

  public void Step() {
    step += 1;
    stepTimer.reset();
  }

  public double getElapsed() {
    return autonTimer.get();
  }

  public double getStepElapsed() {
    return stepTimer.get();
  }

  public int getStep() {
    return step;
  }

  public void Periodic() {
    Logger.getInstance().recordOutput("Auton/Step", step);
    Logger.getInstance().recordOutput("Auton/StepTimer", getStepElapsed());
    Logger.getInstance().recordOutput("Auton/AutonTimer", getElapsed());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addIntegerProperty("Step", () -> step, null);
    builder.addDoubleProperty("StepTimer", () -> getStepElapsed(), null);
    builder.addDoubleProperty("AutonTimer", () -> getElapsed(), null);
  }
}
