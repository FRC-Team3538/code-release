package com.robojackets.auto;

import com.ctre.phoenix6.hardware.TalonFX;
import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.ArmSetpoint;
import com.robojackets.subsystems.swerve.CTRESwerveDrive;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public class CleanSide3gpAuton extends Auton {
  private List<Trajectory> blue_trajectory;
  private List<Trajectory> red_trajectory;
  private List<Trajectory> trajectory;
  private CTRESwerveDrive drive;
  private Arm arm;
  private TalonFX intake;

  public CleanSide3gpAuton(RobotContainer io) {
    this.drive = io.swerve;
    this.arm = io.arm;
    this.intake = io.intakemotor;
    this.blue_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Blue Grid 1 to Piece 1.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Blue Piece 1 to Grid 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Blue Grid 3 to Piece 2.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Blue Piece 2 to Grid 2.path.json"));

    this.red_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Red Grid 1 to Piece 1.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Red Piece 1 to Grid 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Red Grid 3 to Piece 2.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("clear-project-Red Piece 2 to Grid 2.path.json"));

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
  states:
  - arm to high score
  - score
  - trajectory 0, arm to high stow, arm to ground @end, intake pickup
  - trajectory 1, arm to high stow, arm to high score @end
  - score
  - trajectory 2, arm to high stow, arm to ground @end, intake pickup
  - trajectory 3, arm to high score @end
  - score


     */

  @Override
  public void Periodic() {
    super.Periodic();

    if (getStep() == 0) { // arm goto high
      if (arm.GoTo(ArmSetpoint.BIGBOI_HIGH)) {
        Step();
      }

      this.intake.set(0.2);
    } else if (getStep() == 1) { // score
      this.intake.set(-0.3); // negative is spit for cone

      if (getStepElapsed() > 0.2) {
        Step();
      }

      // var holdPose = this.trajectory.get(0).getInitialPose();

      // this.drive.DriveTrajectoryState(
      //     TrajectoryState.builder()
      //         .x(holdPose.getX())
      //         .y(holdPose.getY())
      //         .heading(holdPose.getRotation().getRadians())
      //         .build());
    } else if (getStep() == 2) { // goto piece 1
      if (getStepElapsed() < 1.75) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
        intake.set(0.5); // positive is intake for cone
      }

      var trajectoryIndex = 0;
      if (isStepInit()) {
        drive.DriveTrajectory(trajectory.get(trajectoryIndex));
      }

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 3) { // goto grid 3
      var armGood = false;
      if (getStepElapsed() > 0.25) {
        intake.set(0.15); // positive is intake for cone
      }

      if (getStepElapsed() < 2.5) {
        armGood = arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        armGood = arm.GoTo(ArmSetpoint.BIGBOI_HIGH);
      }

      var trajectoryIndex = 1;
      if (isStepInit()) {
        drive.DriveTrajectory(trajectory.get(trajectoryIndex));
      }

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime() && armGood) {
        Step();
      }
    } else if (getStep() == 4) { // score
      this.intake.set(-0.3); // negative is spit for cone

      // var holdPose = this.trajectory.get(1).getFinalPose();

      // this.drive.DriveTrajectoryState(
      //     TrajectoryState.builder()
      //         .x(holdPose.getX())
      //         .y(holdPose.getY())
      //         .heading(holdPose.getRotation().getRadians())
      //         .build());

      if (getStepElapsed() > 0.2) {
        Step();
      }
    } else if (getStep() == 5) { // goto piece 2
      if (getStepElapsed() < 2) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
        intake.set(-0.2); // negative is intake for cube
      }

      var trajectoryIndex = 2;
      if (isStepInit()) {
        drive.DriveTrajectory(trajectory.get(trajectoryIndex));
      }

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 6) { // goto grid 2
      var armGood = false;
      if (getStepElapsed() < 2.0) {
        armGood = arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        armGood = arm.GoTo(ArmSetpoint.BIGBOI_HIGH);
      }

      var trajectoryIndex = 3;
      if (isStepInit()) {
        drive.DriveTrajectory(trajectory.get(trajectoryIndex));
      }

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime() && armGood) {
        Step();
      }
    } else if (getStep() == 7) { // score
      this.intake.set(0.5); // positive is spit for cube

      // var holdPose = this.trajectory.get(3).getFinalPose();

      // this.drive.DriveTrajectoryState(
      //     TrajectoryState.builder()
      //         .x(holdPose.getX())
      //         .y(holdPose.getY())
      //         .heading(holdPose.getRotation().getRadians())
      //         .build());

      if (getStepElapsed() > 0.2) {
        Step();
      }
    } else {
      drive.Cross();
      arm.GoTo(ArmSetpoint.STOW);
    }
  }
}
