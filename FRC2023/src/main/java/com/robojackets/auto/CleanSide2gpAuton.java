package com.robojackets.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.lib.trajectory.TrajectoryState;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.ArmSetpoint;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public class CleanSide2gpAuton extends Auton {
  private List<Trajectory> blue_trajectory;
  private List<Trajectory> red_trajectory;
  private List<Trajectory> trajectory;
  private SwerveDrive drive;
  private Arm arm;
  private WPI_TalonFX intake;

  public CleanSide2gpAuton(RobotContainer io) {
    this.drive = io.swerve;
    this.arm = io.arm;
    this.intake = io.intakemotor;
    this.blue_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Blue Grid 1 to Piece 1.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Blue Piece 1 to Grid 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Blue Grid 3 to Piece 2.path.json"));

    this.red_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Red Grid 1 to Piece 1.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Red Piece 1 to Grid 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-2gp-Red Grid 3 to Piece 2.path.json"));

    System.out.println("Loaded Auton!");
    // TODO - make this an NT entry ^^
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

    var initialPose = trajectory.get(getStep()).getInitialPose();
    this.drive.resetPose(initialPose);
  }
  /*
   * states:
   * - arm to high score
   * - score
   * - trajectory 0, arm to high stow, arm to ground @end, intake pickup
   * - trajectory 1, arm to high stow, arm to high score @end
   * - score
   * - trajectory 2, arm to high stow, arm to ground @end, intake pickup
   * - trajectory 3, arm to high score @end
   * - score
   *
   *
   */

  @Override
  public void Periodic() {
    super.Periodic();

    if (getStep() == 0) { // arm goto high
      if (arm.GoTo(ArmSetpoint.BIGBOI_HIGH)) {
        Step();
        this.intake.set(0.2);
      }

      this.intake.set(0.2);
    } else if (getStep() == 1) { // score
      this.intake.set(-0.3); // negative is spit for cone

      if (getStepElapsed() > 0.2) {
        Step();
      }

      var holdPose = this.trajectory.get(0).getInitialPose();

      this.drive.DriveTrajectoryState(
          TrajectoryState.builder()
              .x(holdPose.getX())
              .y(holdPose.getY())
              .heading(holdPose.getRotation().getRadians())
              .build());
    } else if (getStep() == 2) { // goto piece 1
      if (getStepElapsed() < 1.90) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
        intake.set(-0.5); // positive is intake for cone
      }

      var trajectoryIndex = 0;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 3) { // goto grid 3
      var armGood = false;
      if (getStepElapsed() > 0.25) {
        intake.set(-0.15); // negative is intake for cube
      }

      if (getStepElapsed() < 2.90) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        armGood = arm.GoTo(ArmSetpoint.HIGH_MILFORD);
      }

      var trajectoryIndex = 1;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime() && armGood) {
        Step();
      }
    } else if (getStep() == 4) { // score
      // this.intake.set(-0.3); // negative is spit for cone

      var holdPose = this.trajectory.get(1).getFinalPose();

      this.drive.DriveTrajectoryState(
          TrajectoryState.builder()
              .x(holdPose.getX())
              .y(holdPose.getY())
              .heading(holdPose.getRotation().getRadians())
              .build());

      if (getStepElapsed() > 0.4) {
        Step();
        this.intake.set(0.3);
      }
    } else if (getStep() == 5) { // goto piece 2
      if (getStepElapsed() < 2.0) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
        intake.set(-0.2); // negative is intake for cube
      }

      var trajectoryIndex = 2;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else {
      drive.Cross();
      arm.GoTo(ArmSetpoint.STOW);
    }
  }
}
