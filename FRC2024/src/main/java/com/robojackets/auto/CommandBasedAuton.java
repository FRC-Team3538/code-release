package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class CommandBasedAuton extends Auton {
  private String name;

  public CommandBasedAuton(String name) {
    this.name = name;
  }

  public Command build(Command sequence) {
    return runOnce(() -> Init()).andThen(run(() -> Periodic())).alongWith(sequence).withName(name);
  }

  public abstract Command build();
}
