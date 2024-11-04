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

public class SequencerTest extends CommandBasedAuton {
  public static final String NAME = "Auton::SequencerTest";
  private Supplier<Command> command;

  private RobotContainer m_io;

  public SequencerTest(RobotContainer io) {
    super(NAME);

    m_io = io;

    var trajectories = Choreo.getTrajectoryGroup("Sequencer Path");
    for (int i = 0; i < trajectories.size(); i++) {
      trajectories.set(i, new AutoMirroredTrajectory(trajectories.get(i)));
    }

    command =
        () -> {
          return sequence(
              print("Beginning Trajectory").deadlineWith(io.swerve.LockWheels()),
              runOnce(() -> io.swerve.resetPose(trajectories.get(0).getInitialPose())),

              // Shoot Preload
              waitUntil(() -> getStep() == 1),

              // Go get Note and come back to shoot
              io.swerve.FollowPath(trajectories.get(0)).andThen(() -> Step()),
              io.swerve.HoldPose(trajectories.get(0).getFinalPose()).until(() -> getStep() == 3),

              // Go get Note and come back to shoot
              io.swerve.FollowPath(trajectories.get(1)).andThen(() -> Step()),
              io.swerve.HoldPose(trajectories.get(1).getFinalPose()).until(() -> getStep() == 5),

              // Get and shoot podium note
              io.swerve.FollowPath(trajectories.get(2)).andThen(() -> Step()),
              io.swerve.HoldPose(trajectories.get(2).getFinalPose()).until(() -> getStep() == 7));
        };
  }

  @Override
  public void Periodic() {
    if (getStep() == 0) {
      // Shooting the Preload object

      m_io.pivot.setAngle(38);
      m_io.shooter.SetShooterRPM(4000);
      if (m_io.pivot.AtAngle(4) && m_io.shooter.AtRPM(100)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }
    if (getStep() == 1) {
      // Driving to first note
      IntakeSequence();
    }

    if (getStep() == 2) {
      m_io.pivot.setAngle(18);
      m_io.shooter.SetShooterRPM(4000);
      if (m_io.pivot.AtAngle(4) && m_io.shooter.AtRPM(100)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 3) {
      IntakeSequence();
    }

    if (getStep() == 4) {
      m_io.pivot.setAngle(30);
      m_io.shooter.SetShooterRPM(4000);
      if (m_io.pivot.AtAngle(4) && m_io.shooter.AtRPM(100)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 5) {
      IntakeSequence();
    }

    if (getStep() == 6) {
      m_io.pivot.setAngle(20);
      m_io.shooter.SetShooterRPM(4000);
      if (m_io.pivot.AtAngle(4) && m_io.shooter.AtRPM(100)) {
        m_io.shooter.setFeeder(0.3, true);
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 7) {
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
