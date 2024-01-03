package com.robojackets.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.ArmSetpoint;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public class FastBumpSide3gpAuton extends Auton {
  private List<Trajectory> blue_trajectory;
  private List<Trajectory> red_trajectory;
  private List<Trajectory> trajectory;
  private SwerveDrive drive;
  private Arm arm;
  private WPI_TalonFX intake;

  public FastBumpSide3gpAuton(RobotContainer io) {
    this.drive = io.swerve;
    this.arm = io.arm;
    this.intake = io.intakemotor;
    this.blue_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Blue Grid 9 to Piece 4.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Blue Piece 4 to Grid 8.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Blue Grid 8 to Piece 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Blue Piece 3 to Grid 8.path.json"));

    this.red_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Red Grid 9 to Piece 4.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Red Piece 4 to Grid 8.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Red Grid 8 to Piece 3.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("fast-bump-project-Red Piece 3 to Grid 8.path.json"));

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
  - suck cone
  - spit cone, trajectory 0, arm to high stow, arm to ground @end, intake pickup cube
  - trajectory 1, arm to high stow, arm to mid score @end
  - score
  - trajectory 2, arm to high stow, arm to ground @end, intake pickup
  - trajectory 3, arm to high stow, intake regular speed, arm to stow @end
  - poop


     */

  @Override
  public void Periodic() {
    super.Periodic();

    if (getStep() == 0) {
      this.intake.set(0.2); // Intaking to get da ziptie out

      if (getStepElapsed() > 0.2) {
        Step();
      }
    } else if (getStep() == 1) { // goto piece 4
      if (getStepElapsed() < 0.5) {
        this.intake.set(-1.0); // negative is spit for cone
      } else if (getStepElapsed() < 2.5) {
        this.intake.set(-0.1); // idle
      } else {
        this.intake.set(-0.5); // negative is intake for cube
      }

      if (getStepElapsed() < 2.25) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
      }

      var trajectoryIndex = 0;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }

    } else if (getStep() == 2) { // goto grid 8
      this.intake.set(-0.2); // negative is intake for cube
      if (getStepElapsed() < 2.25) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.MID);
      }

      var trajectoryIndex = 1;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()
          && this.arm.AtSetpoint()) {
        Step();
      }
    } else if (getStep() == 3) { // goto piece 3
      if (getStepElapsed() < 0.5) {
        this.intake.set(0.6); // positive is spit for cube
      } else if (getStepElapsed() < 2.3) {
        this.intake.set(0.1); // idle da spit for intake
      } else {
        this.intake.set(-0.5); // negative is intake for cube
      }

      if (getStepElapsed() < 0.5) {
        this.arm.GoTo(ArmSetpoint.MID);
      } else if (getStepElapsed() < 2) {
        this.arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        this.arm.GoTo(ArmSetpoint.GROUND);
      }

      var trajectoryIndex = 2;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 4) { // goto grid 8
      this.intake.set(-0.2); // negative is intake for cube
      if (getStepElapsed() < 3.0) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.STOW);
      }

      var trajectoryIndex = 3;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 5) { // score
      this.intake.set(0.4); // positive is spit for cube
      drive.Cross();

      if (getStepElapsed() > 0.5) {
        Step();
      }
    } else {
      drive.Cross();
      arm.GoTo(ArmSetpoint.STOW);
      intake.set(0.15);
    }
  }
}
