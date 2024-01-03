package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.List;

public class FollowCustomTrajectoryAuton extends Auton {
  private List<Trajectory> trajectory;
  private SwerveDrive drive;

  public FollowCustomTrajectoryAuton(RobotContainer io, List<Trajectory> trajectories) {
    this.drive = io.swerve;
    this.trajectory = trajectories;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void Init() {
    // Must call
    super.Init();
    this.drive.resetPose(trajectory.get(getStep()).getInitialPose());
  }

  @Override
  public void Periodic() {
    if (getStep() >= trajectory.size()) {
      drive.DriveFieldCentric(new ChassisSpeeds());
      return;
    }

    drive.DriveTrajectoryState(trajectory.get(getStep()).sample(getStepElapsed()));

    if (getStepElapsed() > trajectory.get(getStep()).getTotalTime()) {
      Step();
    }
  }
}
