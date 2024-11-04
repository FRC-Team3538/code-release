package com.robojackets.auto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public abstract class Auton implements Sendable {
  private Timer autonTimer = new Timer();
  private int lastStep = -1;
  private int step = 0;
  private boolean stepInit = true;
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

  public boolean isStepInit() {
    return stepInit;
  }

  public void Periodic() {
    Logger.recordOutput("Auton/Step", step);
    Logger.recordOutput("Auton/StepTimer", getStepElapsed());
    Logger.recordOutput("Auton/AutonTimer", getElapsed());

    stepInit = lastStep != step;
    lastStep = step;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addIntegerProperty("Step", () -> step, null);
    builder.addDoubleProperty("StepTimer", () -> getStepElapsed(), null);
    builder.addDoubleProperty("AutonTimer", () -> getElapsed(), null);
  }
}
