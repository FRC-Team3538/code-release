package com.robojackets.auto;

import edu.wpi.first.util.sendable.SendableBuilder;

public class EmptyAuton extends Auton {

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
  public void Periodic() {}
}
