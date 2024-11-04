package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import edu.wpi.first.wpilibj2.command.Command;

public class EmptyCommandAuton extends CommandBasedAuton {

  public EmptyCommandAuton() {
    super("Auton::Empty");
  }

  @Override
  public Command build() {
    return print("Beginning Empty Auton");
  }
}
