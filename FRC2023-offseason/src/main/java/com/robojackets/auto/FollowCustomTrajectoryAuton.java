package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.subsystems.swerve.CTRESwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public class FollowCustomTrajectoryAuton extends Auton {
  private List<Trajectory> red_trajectory;
  private List<Trajectory> blue_trajectory;
  private List<Trajectory> trajectory;
  private CTRESwerveDrive drive;

  public FollowCustomTrajectoryAuton(
      RobotContainer io, List<Trajectory> redTrajectories, List<Trajectory> blueTrajectories) {
    this.drive = io.swerve;
    this.red_trajectory = redTrajectories;
    this.blue_trajectory = blueTrajectories;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void Init() {
    // Must call
    super.Init();

    if (trajectory == null) {
      if (DriverStation.getAlliance().equals(Alliance.Red)) {
        trajectory = red_trajectory;
      } else {
        trajectory = blue_trajectory;
      }
      red_trajectory = null;
      blue_trajectory = null;
    }

    this.drive.resetPose(trajectory.get(getStep()).getInitialPose());
    this.drive.DriveTrajectory(trajectory.get(getStep()));
  }

  @Override
  public void Periodic() {
    // Must call
    super.Periodic();

    if (getStep() >= trajectory.size()) {
      drive.DriveFieldCentric(new ChassisSpeeds());
      return;
    }

    if (isStepInit()) {
      drive.DriveTrajectory(trajectory.get(getStep()));
    }

    if (getStepElapsed() > trajectory.get(getStep()).getTotalTime()) {
      Step();
    }
  }
}
