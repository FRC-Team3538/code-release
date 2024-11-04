package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.choreo.lib.Choreo;
import com.robojackets.RobotContainer;
import com.robojackets.lib.choreo.AutoMirroredTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class South4RedSwitcheraoo extends CommandBasedAuton {
  public static final String NAME = "Auton::FollowTrajectory";
  private Supplier<Command> command;

  private final double PREAIM_DURATION = 1.0; // seconds
  private boolean preAim = false;
  private boolean firsttimeshot = false;

  private RobotContainer m_io;

  public South4RedSwitcheraoo(RobotContainer io) {
    super(NAME);

    m_io = io;

    var trajectories = Choreo.getTrajectoryGroup("south 4 red note switch");
    for (int i = 0; i < trajectories.size(); i++) {
      trajectories.set(i, new AutoMirroredTrajectory(trajectories.get(i)));
    }

    command =
        () -> {
          return sequence(
              print("Beginning Trajectory").deadlineWith(io.swerve.LockWheels()),
              runOnce(() -> io.swerve.resetPose(trajectories.get(0).getInitialPose())),
              waitUntil(() -> getStep() == 1),
              parallel(
                  waitSeconds(trajectories.get(0).getTotalTime() - PREAIM_DURATION)
                      .andThen(
                          () -> {
                            preAim = true;
                          }),
                  io.swerve.FollowPath(trajectories.get(0)).andThen(() -> Step())),
              io.swerve.HoldPose(trajectories.get(0).getFinalPose()).until(() -> getStep() == 3),
              parallel(
                  waitSeconds(trajectories.get(1).getTotalTime() - PREAIM_DURATION)
                      .andThen(
                          () -> {
                            preAim = true;
                          }),
                  io.swerve.FollowPath(trajectories.get(1)).andThen(() -> Step())),
              io.swerve.HoldPose(trajectories.get(1).getFinalPose()).until(() -> getStep() == 5),
              io.swerve.FollowPath(trajectories.get(2)).andThen(() -> Step()),
              io.swerve.LockWheels());
        };
  }

  @Override
  public void Periodic() {
    if (getStep() == 0) {
      // Shooting the Preload object
      m_io.elevator.PositionCommand(2.0);
      m_io.pivot.setAngle(38);
      m_io.shooter.SetShooterRPM(2700);
      if ((m_io.pivot.AtAngle(2.0) && m_io.shooter.AtRPM(150)) || firsttimeshot) {
        m_io.shooter.setFeeder(1.0, true);
        firsttimeshot = true;
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }
    if (getStep() == 1) {
      // Driving to first note
      firsttimeshot = false;
      m_io.elevator.PositionCommand(0.0);
      if (!preAim) {
        IntakeSequence();
      } else if (m_io.shooter.NoteDetected()) {
        Aim(0.3);
      }
    }

    if (getStep() == 2) {
      preAim = false;
      m_io.shooter.SetShooterRPM(m_io.shooter.getInterpolateVals(m_io.visiondistance()));
      m_io.pivot.setAngle(m_io.pivot.getInterpolateVals(m_io.visiondistance()) + 0.5);
      if ((m_io.pivot.AtAngle(0.6) && m_io.shooter.AtRPM(150)) || firsttimeshot) {
        m_io.shooter.setFeeder(1.0, true);
        firsttimeshot = true;
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 3) {
      firsttimeshot = false;
      m_io.elevator.PositionCommand(0.0);
      if (!preAim) {
        IntakeSequence();
      } else if (m_io.shooter.NoteDetected()) {
        Aim(0.5);
      }
    }

    if (getStep() == 4) {
      preAim = false;
      m_io.shooter.SetShooterRPM(m_io.shooter.getInterpolateVals(m_io.visiondistance()));
      m_io.pivot.setAngle(m_io.pivot.getInterpolateVals(m_io.visiondistance()) + 0.5);
      if ((m_io.pivot.AtAngle(0.6) && m_io.shooter.AtRPM(150)) || firsttimeshot) {
        m_io.shooter.setFeeder(1.0, true);
        firsttimeshot = true;
        if (m_io.shooter.NoteNotDetected()) { // old if condition getStepElapsed() > 2.0
          Step();
        }
      }
    }

    if (getStep() == 5) {
      firsttimeshot = false;
      m_io.elevator.PositionCommand(0.0);
      m_io.pivot.setAngle(1.5);
      m_io.shooter.SetShooterRPM(0);
      m_io.intake.run(0.0);
      m_io.director.setDirectortoStop();
      m_io.shooter.setFeeder(0.0, false);
    }
  }

  public Command build() {
    return build(command.get());
  }

  public void IntakeSequence() {
    m_io.intake.run(0.5);
    m_io.director.setDirectortoShooter(1.0);
    m_io.shooter.setFeeder(0.3, false);
    m_io.pivot.setAngle(1.5);
    m_io.shooter.SetShooterRPM(0);
  }

  public void Aim(double pitchoffset) {
    m_io.shooter.SetShooterRPM(m_io.shooter.getInterpolateVals(m_io.visiondistance()));
    m_io.pivot.setAngle(m_io.pivot.getInterpolateVals(m_io.visiondistance()) + pitchoffset);
  }
}
