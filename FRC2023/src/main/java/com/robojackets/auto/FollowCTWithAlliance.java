package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.List;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class FollowCTWithAlliance extends Auton {
  private List<Trajectory> trajectory;
  private SwerveDrive drive;
  private Arm arm;

  public FollowCTWithAlliance(RobotContainer io, List<Trajectory> trajectories) {
    this.drive = io.swerve;
    this.arm = io.arm;
    this.trajectory = trajectories;

    System.out.println("Loaded Auton!");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void Init() {
    // Must call
    super.Init();

    var initialPose = AllianceFlipUtil.apply(trajectory.get(getStep()).getInitialPose());
    this.drive.resetPose(initialPose);
  }

  @Override
  public void Periodic() {
    super.Periodic();

    this.arm.GoTo(1, 132.0);

    if (getStep() >= trajectory.size()) {
      drive.Cross();
      return;
    }

    var sample = AllianceFlipUtil.apply(trajectory.get(getStep()).sample(getStepElapsed()));

    drive.DriveTrajectoryState(sample);

    if (getStepElapsed() > trajectory.get(getStep()).getTotalTime()) {
      Step();
    }
  }
}
