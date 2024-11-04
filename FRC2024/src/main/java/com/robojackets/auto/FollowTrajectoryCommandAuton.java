package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import com.choreo.lib.ChoreoTrajectory;
import com.robojackets.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class FollowTrajectoryCommandAuton extends CommandBasedAuton {
  public static final String NAME = "Auton::FollowTrajectory";
  private Supplier<Command> command;

  public FollowTrajectoryCommandAuton(
      RobotContainer io, Supplier<ChoreoTrajectory> trajectorySupplier) {
    super(NAME);

    command =
        () -> {
          var trajectory = trajectorySupplier.get();
          return print("Beginning Trajectory")
              .deadlineWith(io.swerve.LockWheels())
              .andThen(() -> io.swerve.resetPose(trajectory.getInitialPose()))
              .andThen(io.swerve.FollowPath(trajectory))
              .andThen(io.swerve.HoldPose(trajectory.getFinalPose()));
        };
  }

  public Command build() {
    return build(command.get());
  }
}
