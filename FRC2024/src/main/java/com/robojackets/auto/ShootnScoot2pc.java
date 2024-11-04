package com.robojackets.auto;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.robojackets.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class ShootnScoot2pc extends CommandBasedAuton {
  public static final String NAME = "Auton::ShootnScoot2pc";
  private Supplier<Command> command;

  private RobotContainer m_io;

  public ShootnScoot2pc(RobotContainer io) {
    super(NAME);

    m_io = io;

    command =
        () -> {
          return sequence(print("Beginning ShootnScoot2pc (I hate command based)"));
        };
  }

  @Override
  public void Periodic() {
    if (getStep() == 0) {
      // Shooting the Preload object
      ShootSequence(38, 4000);
      if (getStepElapsed() > 2.0) {
        Step();
      }
    }
    if (getStep() == 1) {
      // Driving to first note
      IntakeSequence();
      m_io.swerve.RobotCentricDrive(new ChassisSpeeds(0.5, 0.0, 0.0));

      if (getStepElapsed() > 4.0) {
        Step();
      }
    }

    if (getStep() == 2) {
      // Driving Back to Subwoofer
      IntakeSequence();
      m_io.swerve.RobotCentricDrive(new ChassisSpeeds(-0.5, 0.0, 0.0));

      if (getStepElapsed() > 4.0) {
        Step();
      }
    }

    if (getStep() == 3) {
      ShootSequence(38, 4000);
      m_io.swerve.RobotCentricDrive(new ChassisSpeeds(0, 0, 0));
      if (getStepElapsed() > 2.0) {
        Step();
      }
    }

    if (getStep() == 4) {
      m_io.swerve.RobotCentricDrive(new ChassisSpeeds(0, 0, 0));
      StopShootSequence();
      StopIntakeSequence();
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
    }
  }

  public void StopShootSequence() {
    m_io.pivot.setAngle(0);
    m_io.shooter.SetShooterRPM(0);
  }

  public void IntakeSequence() {
    m_io.intake.run(0.5);
    m_io.director.setDirectortoShooter(0.9);
    m_io.shooter.setFeeder(0.3, false);
    m_io.pivot.setAngle(0);
    m_io.shooter.SetShooterRPM(0);
  }

  public void StopIntakeSequence() {
    m_io.intake.run(0.0);
    m_io.director.setDirectortoStop();
    m_io.shooter.setFeeder(0.0, false);
  }
}
