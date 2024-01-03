package com.robojackets.lib.trajectory;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class PPTrajectory extends Trajectory {
  private PathPlannerTrajectory trajectory;

  @Override
  public TrajectoryState sample(double timestamp) {
    var sample = (PathPlannerState) trajectory.sample(timestamp);

    return TrajectoryState.builder()
        .timestamp(sample.timeSeconds)
        .x(sample.poseMeters.getX())
        .y(sample.poseMeters.getY())
        .heading(sample.holonomicRotation.getRadians())
        .velocityX(sample.velocityMetersPerSecond * sample.poseMeters.getRotation().getCos())
        .velocityY(sample.velocityMetersPerSecond * sample.poseMeters.getRotation().getSin())
        .angularVelocity(sample.holonomicAngularVelocityRadPerSec)
        .build();
  }

  @Override
  public Pose2d getInitialPose() {
    return trajectory.getInitialHolonomicPose();
  }

  @Override
  public double getTotalTime() {
    return trajectory.getTotalTimeSeconds();
  }
}
