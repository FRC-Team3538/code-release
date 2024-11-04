package com.robojackets.lib.choreo;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoMirroredTrajectory extends ChoreoTrajectory {
  private final ChoreoTrajectory trajectory;

  public AutoMirroredTrajectory(ChoreoTrajectory trajectory) {
    this.trajectory = trajectory;
  }

  private boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  @Override
  public ChoreoTrajectoryState getInitialState() {
    if (shouldFlip()) {
      return trajectory.getInitialState().flipped();
    }
    return trajectory.getInitialState();
  }

  @Override
  public ChoreoTrajectoryState getFinalState() {
    if (shouldFlip()) {
      return trajectory.getFinalState().flipped();
    }
    return trajectory.getFinalState();
  }

  @Override
  public ChoreoTrajectoryState sample(double timestamp) {
    return trajectory.sample(timestamp, shouldFlip());
  }

  @Override
  public ChoreoTrajectoryState sample(double timestamp, boolean mirrorForRedAlliance) {
    return trajectory.sample(timestamp, mirrorForRedAlliance);
  }

  @Override
  public Pose2d getInitialPose() {
    return getInitialState().getPose();
  }

  @Override
  public Pose2d getFinalPose() {
    return getFinalState().getPose();
  }

  @Override
  public double getTotalTime() {
    return trajectory.getTotalTime();
  }

  @Override
  public Pose2d[] getPoses() {
    if (shouldFlip()) {
      return trajectory.flipped().getPoses();
    }

    return trajectory.getPoses();
  }

  @Override
  public ChoreoTrajectory flipped() {
    return trajectory.flipped();
  }
}
