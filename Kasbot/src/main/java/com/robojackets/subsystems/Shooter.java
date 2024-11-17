package com.robojackets.subsystems;

import static com.robojackets.lib.UnitsUtils.abs;
import static com.robojackets.subsystems.Shooter.SpinDirection.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.robojackets.lib.RJLog;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter extends SubsystemBase {
  public static enum SpinDirection {
    Neither,
    Clockwise,
    CounterClockwise;
  }

  private final TalonFX leftMotor = new TalonFX(21, "*");
  private final TalonFX rightMotor = new TalonFX(20, "*");

  TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);

  MotionMagicVelocityTorqueCurrentFOC leftVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  MotionMagicVelocityTorqueCurrentFOC rightVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

  // SysId is built for arguments in Volts, but we want to use Amps instead.
  // So, we pretend our arguments in Amps are actually in Volts.
  SysIdRoutine shooterRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(10).per(Second), // Amps per Second
              Volts.of(80), // Amps
              Seconds.of(10),
              state -> SignalLogger.writeString("shooter-sysid-state", state.toString())),
          new SysIdRoutine.Mechanism(
              amps -> {
                leftMotor.setControl(currentControl.withOutput(amps.in(Volts)));
                rightMotor.setControl(currentControl.withOutput(amps.in(Volts)));
              },
              null,
              this,
              "shooter"));

  public Shooter() {
    var motorConfig = new TalonFXConfiguration();

    // shared config

    // Exponential Profile constants
    // Note - values come from SysId w/ Voltage as input
    // Values should be greater than the estimated Feedforward constants (from sysid) in order for
    // the profile to be feasible
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    motorConfig.MotionMagic.MotionMagicAcceleration = 100;
    motorConfig.MotionMagic.MotionMagicExpo_kV = 0.14;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 0.03;

    // left config
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Slot 0 is for TorqueCurrent-based control
    // Note - values come from sysid w/ TorqueCurrentAmps provided in place of Voltage
    motorConfig.Slot0.kS = 16.2722; // Amps
    motorConfig.Slot0.kV = 0.34568; // Amps / (motion profile velocity)
    motorConfig.Slot0.kA = 0.75615; // Amps / (motion profile acceleration)
    motorConfig.Slot0.kP = 13.595; // Amps / (velocity error)

    leftMotor.getConfigurator().apply(motorConfig);

    // right config
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Slot 0 is for TorqueCurrent-based control
    // Note - values come from sysid w/ TorqueCurrentAmps provided in place of Voltage
    motorConfig.Slot0.kS = 19.559; // Amps
    motorConfig.Slot0.kV = 0.816662; // Amps / (motion profile velocity)
    motorConfig.Slot0.kA = 0.9154; // Amps / (motion profile acceleration)
    motorConfig.Slot0.kP = 15; // Amps / (velocity error)

    rightMotor.getConfigurator().apply(motorConfig);

    Preferences.initDouble("Shooter/Spin_Ratio", 1);
    setDefaultCommand(primeCommand(RotationsPerSecond.of(0), Neither));
  }

  public Command primeCommand(Measure<Velocity<Angle>> shotVelocity, SpinDirection spinDirection) {
    return run(() -> prime(shotVelocity, spinDirection)).withName("Shooter::Prime");
  }

  public void prime(Measure<Velocity<Angle>> shotVelocity, SpinDirection spinDirection) {
    Measure<Velocity<Angle>> leftVelocity;
    Measure<Velocity<Angle>> rightVelocity;

    var spin = Preferences.getDouble("Shooter/Spin_Ratio", 1);

    if (abs(shotVelocity).lt(RotationsPerSecond.of(1e-4))) {
      leftVelocityControl.Velocity = 0;
      rightVelocityControl.Velocity = 0;

      leftMotor.setControl(new CoastOut());
      rightMotor.setControl(new CoastOut());
      return;
    }

    switch (spinDirection) {
      case Clockwise:
        // For clockwise spin left wheels need to spin faster than right wheels
        leftVelocity = shotVelocity.times(spin);
        rightVelocity = shotVelocity;
        break;
      case CounterClockwise:
        // For counterclockwise spin right wheels need to spin faster than left wheels
        leftVelocity = shotVelocity;
        rightVelocity = shotVelocity.times(spin);
        break;
      case Neither:
      default:
        leftVelocity = shotVelocity;
        rightVelocity = shotVelocity;
        break;
    }

    leftMotor.setControl(leftVelocityControl.withVelocity(leftVelocity.in(RotationsPerSecond)));
    rightMotor.setControl(rightVelocityControl.withVelocity(rightVelocity.in(RotationsPerSecond)));
  }

  @Override
  public void periodic() {
    RJLog.log("Shooter/leftRPM", getLeftVelocity());
    RJLog.log("Shooter/leftTargetRPM", getLeftTargetVelocity());
    RJLog.log("Shooter/rightRPM", getRightVelocity());
    RJLog.log("Shooter/rightTargetRPM", getRightTargetVelocity());

    RJLog.log("Shooter/leftError", getLeftTargetVelocity().minus(getLeftVelocity()));
    RJLog.log("Shooter/rightError", getRightTargetVelocity().minus(getRightVelocity()));
  }

  public Measure<Velocity<Angle>> getLeftVelocity() {
    return RotationsPerSecond.of(leftMotor.getVelocity().getValue());
  }

  public Measure<Velocity<Angle>> getRightVelocity() {
    return RotationsPerSecond.of(rightMotor.getVelocity().getValue());
  }

  public Measure<Velocity<Angle>> getLeftTargetVelocity() {
    return RotationsPerSecond.of(leftVelocityControl.Velocity);
  }

  public Measure<Velocity<Angle>> getRightTargetVelocity() {
    return RotationsPerSecond.of(rightVelocityControl.Velocity);
  }

  public boolean hasNonZeroSetpoint() {
    return getLeftTargetVelocity().gt(RotationsPerSecond.of(0))
        && getRightTargetVelocity().gt(RotationsPerSecond.of(0));
  }

  public boolean isReady(Measure<Velocity<Angle>> tolerance) {
    RJLog.log("Shooter/Tolerance", RotationsPerSecond.of(tolerance.in(RotationsPerSecond)));
    return abs(getLeftVelocity().minus(getLeftTargetVelocity())).lt(tolerance)
        && abs(getRightVelocity().minus(getRightTargetVelocity())).lt(tolerance);
  }

  public static Measure<Velocity<Angle>> CUTOFF_SPEED = RotationsPerSecond.of(50);
  public static Measure<Velocity<Angle>> STOPPED = RotationsPerSecond.of(0.1);

  public Command sysidShooter() {
    return shooterRoutine
        .quasistatic(Direction.kForward)
        .until(
            () ->
                abs(getLeftVelocity()).gt(CUTOFF_SPEED) && abs(getRightVelocity()).gt(CUTOFF_SPEED))
        .andThen(
            Commands.run(
                    () -> {
                      leftMotor.stopMotor();
                      rightMotor.stopMotor();
                    })
                .until(
                    () ->
                        abs(getLeftVelocity()).lt(STOPPED) && abs(getRightVelocity()).lt(STOPPED)))
        .andThen(
            shooterRoutine
                .quasistatic(Direction.kReverse)
                .until(
                    () ->
                        abs(getLeftVelocity()).gt(CUTOFF_SPEED)
                            && abs(getRightVelocity()).gt(CUTOFF_SPEED)))
        .andThen(
            Commands.run(
                    () -> {
                      leftMotor.stopMotor();
                      rightMotor.stopMotor();
                    })
                .until(
                    () ->
                        abs(getLeftVelocity()).lt(STOPPED) && abs(getRightVelocity()).lt(STOPPED)))
        .andThen(
            shooterRoutine
                .dynamic(Direction.kForward)
                .until(
                    () ->
                        abs(getLeftVelocity()).gt(CUTOFF_SPEED)
                            && abs(getRightVelocity()).gt(CUTOFF_SPEED)))
        .andThen(
            Commands.run(
                    () -> {
                      leftMotor.stopMotor();
                      rightMotor.stopMotor();
                    })
                .until(
                    () ->
                        abs(getLeftVelocity()).lt(STOPPED) && abs(getRightVelocity()).lt(STOPPED)))
        .andThen(
            shooterRoutine
                .dynamic(Direction.kReverse)
                .until(
                    () ->
                        abs(getLeftVelocity()).gt(CUTOFF_SPEED)
                            && abs(getRightVelocity()).gt(CUTOFF_SPEED)))
        .andThen(
            Commands.run(
                    () -> {
                      leftMotor.stopMotor();
                      rightMotor.stopMotor();
                    })
                .until(
                    () ->
                        abs(getLeftVelocity()).lt(STOPPED) && abs(getRightVelocity()).lt(STOPPED)))
        .withName("Shooter::SysIdShooter");
  }
}
