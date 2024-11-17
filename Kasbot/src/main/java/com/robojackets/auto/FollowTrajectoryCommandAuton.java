package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.choreo.lib.ChoreoTrajectory;
import com.robojackets.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Set;

public class FollowTrajectoryCommandAuton extends SequentialCommandGroup {
  public static final String NAME = "Auton::FollowTrajectory";

  public FollowTrajectoryCommandAuton(Drive drive, ChoreoTrajectory trajectory) {
    super(
        print("Beginning Trajectory"),
        either(
            defer(
                () ->
                    runOnce(() -> drive.resetPose(trajectory.getInitialPose()))
                        .andThen(drive.FollowPath(trajectory))
                        .andThen(drive.HoldPose(trajectory.getFinalPose())),
                Set.of(drive)),
            drive.LockWheels(),
            () -> trajectory != null));
    setName(NAME);
  }
}
