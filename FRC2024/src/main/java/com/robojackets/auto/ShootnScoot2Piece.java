package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.choreo.lib.Choreo;
import com.robojackets.RobotContainer;
import com.robojackets.lib.choreo.AutoMirroredTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class ShootnScoot2Piece extends CommandBasedAuton {
  public static final String NAME = "Auton::FollowTrajectory";
  private Supplier<Command> command;

  private RobotContainer m_io;

  public ShootnScoot2Piece(RobotContainer io) {
    super(NAME);

    m_io = io;

    var trajectories = Choreo.getTrajectoryGroup("ShootnScoot");
    for (int i = 0; i < trajectories.size(); i++) {
      trajectories.set(i, new AutoMirroredTrajectory(trajectories.get(i)));
    }

    command =
        () -> {
          return sequence(
              print("Beginning Trajectory").deadlineWith(io.swerve.LockWheels()),
              runOnce(() -> io.swerve.resetPose(trajectories.get(0).getInitialPose())),
              waitUntil(() -> getStep() == 1),
              io.swerve.FollowPath(trajectories.get(0)).andThen(() -> Step()),
              io.swerve.HoldPose(trajectories.get(0).getFinalPose()).until(() -> getStep() == 3),
              io.swerve.LockWheels());
        };
  }

  @Override
  public void Periodic() {
    if (getStep() == 0) {
      // Shooting the Preload object
      m_io.elevator.PositionCommand(2.0);
      m_io.pivot.setAngle(35);
      m_io.shooter.SetShooterRPM(3000);
      if (m_io.pivot.AtAngle(2) && m_io.shooter.AtRPM(150)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }
    if (getStep() == 1) {
      // Driving to first note
      m_io.elevator.PositionCommand(2.0);
      IntakeSequence();
    }

    if (getStep() == 2) {
      m_io.shooter.SetShooterRPM(m_io.shooter.getInterpolateVals(m_io.visiondistance()));
      m_io.pivot.setAngle(m_io.pivot.getInterpolateVals(m_io.visiondistance()));
      if (m_io.pivot.AtAngle(0.6) && m_io.shooter.AtRPM(150)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 3) {
      m_io.elevator.PositionCommand(1.5);
      m_io.pivot.setAngle(0);
      m_io.shooter.SetShooterRPM(0);
      m_io.intake.run(0.0);
      m_io.director.setDirectortoStop();
      m_io.shooter.setFeeder(0.0, false);
    }
  }

  public Command build() {
    return build(command.get());
  }

  public void ShootSequence(double angle, double rpm) {
    m_io.pivot.setAngle(angle);
    m_io.shooter.SetShooterRPM(rpm);

    if (m_io.pivot.AtAngle(4) && m_io.shooter.AtRPM(100)) {
      m_io.shooter.setFeeder(0.3, true);
      Step();
    }
  }

  public void IntakeSequence() {
    m_io.intake.run(0.5);
    m_io.director.setDirectortoShooter(0.9);
    m_io.shooter.setFeeder(0.3, false);
    m_io.pivot.setAngle(0);
    m_io.shooter.SetShooterRPM(0);
  }
}
