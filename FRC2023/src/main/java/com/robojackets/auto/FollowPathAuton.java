package com.robojackets.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.robojackets.RobotContainer;
import com.robojackets.lib.UnitConversion;
import com.robojackets.lib.trajectory.PPTrajectory;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.List;

public class FollowPathAuton extends Auton {
  private List<PPTrajectory> trajectory;
  private SwerveDrive drive;

  public FollowPathAuton(RobotContainer io, String pathName) {
    this.drive = io.swerve;
    this.trajectory =
        PathPlanner.loadPathGroup(
                pathName,
                new PathConstraints(
                    6 * UnitConversion.METERS_PER_FOOT, 6 * UnitConversion.METERS_PER_FOOT))
            .stream()
            .map(traj -> new PPTrajectory(traj))
            .toList();
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
