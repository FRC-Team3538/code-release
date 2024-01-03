package com.robojackets.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.Trajectory;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.lib.trajectory.TrajectoryState;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.ArmSetpoint;
import com.robojackets.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public class BumpSide2gpBalance extends Auton {
  private List<Trajectory> blue_trajectory;
  private List<Trajectory> red_trajectory;
  private List<Trajectory> trajectory;
  private SwerveDrive drive;
  private Arm arm;
  private WPI_TalonFX intake;

  public BumpSide2gpBalance(RobotContainer io) {
    this.drive = io.swerve;
    this.arm = io.arm;
    this.intake = io.intakemotor;
    this.blue_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Blue Grid 9 to Piece 4.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Blue Piece 4 to Grid 8.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Blue Grid 8 to Bridge.path.json"));

    this.red_trajectory =
        List.of(
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Red Grid 9 to Piece 4.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Red Piece 4 to Grid 8.path.json"),
            TrajectoryManager.getInstance()
                .getTrajectory("bump-project-Red Grid 8 to Bridge.path.json"));

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
  - trajectory 0, arm to high stow, arm to ground @end, intake pickup cube
  - trajectory 1, arm to high stow, arm to high score @end
  - score
  - trajectory 2, arm to high stow, arm to ground @end, intake pickup
  - arm to stow, intake regular speed


     */

  @Override
  public void Periodic() {
    super.Periodic();

    if (getStep() == 0) { // arm goto high
      if (arm.GoTo(ArmSetpoint.HIGH_MUSKEGON)) {
        Step();
      }

      this.intake.set(0.2);
    } else if (getStep() == 1) { // score
      this.intake.set(-0.5); // negative is spit for cone

      if (getStepElapsed() > 0.3) {
        Step();
      }

    } else if (getStep() == 2) { // goto piece 4
      if (getStepElapsed() < 2.5) {
        arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        arm.GoTo(ArmSetpoint.GROUND);
        intake.set(-0.3); // negative is intake for cube
      }

      var trajectoryIndex = 0;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 3) { // goto grid 8
      var armGood = false;
      if (getStepElapsed() > 0.25) {
        intake.set(-0.15); // negative is intake for cube
      }

      if (getStepElapsed() < 3.25) {
        armGood = arm.GoTo(ArmSetpoint.HIGH_STOW);
      } else {
        armGood = arm.GoTo(ArmSetpoint.HIGH_MUSKEGON);
      }

      var trajectoryIndex = 1;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime() && armGood) {
        Step();
      }
    } else if (getStep() == 4) { // score
      this.intake.set(1.0); // positive is spit for cube

      var holdPose = this.trajectory.get(1).getFinalPose();

      this.drive.DriveTrajectoryState(
          TrajectoryState.builder()
              .x(holdPose.getX())
              .y(holdPose.getY())
              .heading(holdPose.getRotation().getRadians())
              .build());

      if (getStepElapsed() > 0.3) {
        Step();
      }
    } else if (getStep() == 5) { // goto bridge
      arm.GoTo(ArmSetpoint.HIGH_STOW);

      var trajectoryIndex = 2;
      var sample = trajectory.get(trajectoryIndex).sample(getStepElapsed());

      drive.DriveTrajectoryState(sample);

      if (getStepElapsed() > trajectory.get(trajectoryIndex).getTotalTime()) {
        Step();
      }
    } else if (getStep() == 6) {
      arm.GoTo(ArmSetpoint.STOW);
      double pitch = -drive.ioInputs.getPitchRadians() * 180.0 / Math.PI;
      double kP = 0.017;
      double vx = pitch * kP;

      boolean robotIsLevel = Math.abs(drive.ioInputs.getPitchRadians() * 180.0 / Math.PI) < 2.0;
      if (!robotIsLevel) {
        drive.DriveRobotCentric(new ChassisSpeeds(vx, 0.0, 0.0));
      }
    } else {
      drive.Cross();
      arm.GoTo(ArmSetpoint.STOW);
      intake.set(0.15);
    }
  }
}
