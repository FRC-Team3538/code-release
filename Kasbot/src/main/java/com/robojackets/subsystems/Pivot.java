package com.robojackets.subsystems;

import static com.robojackets.lib.UnitsUtils.abs;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.robojackets.lib.RJLog;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;

public class Pivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(22, "*");

  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
  private final DutyCycleOut manualControl = new DutyCycleOut(0);
  private final VoltageOut sysidControl = new VoltageOut(0);

  private final StatusSignal<Double> pitchSignal = motor.getPosition();
  private final StatusSignal<Double> velocitySignal = motor.getVelocity();

  SysIdRoutine pivotRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(6),
              Seconds.of(10),
              state -> SignalLogger.writeString("pivot-sysid-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> {
                motor.setControl(sysidControl.withOutput(volts.in(Volts)));
              },
              null,
              this,
              "pivot"));

  // This value comes from feedforward; if the pivot is at 0 at hard stop*, then the kG feedforward
  // component only works when fed an angle ~23 degrees lower.
  private static final Measure<Angle> PIVOT_OFFSET = Rotations.of(-0.065969);
  // hard stop
  private static final Measure<Angle> PIVOT_MIN = Rotations.of(0).plus(PIVOT_OFFSET);
  // couple degrees before upper frame
  private static final Measure<Angle> PIVOT_MAX = Degrees.of(63).plus(PIVOT_OFFSET);

  private static final Measure<Angle> NEAR_ZERO = Rotations.of(5e-3);

  public Pivot() {
    var motorConfig = new TalonFXConfiguration();

    // TODO - teach the feedforward setup
    motorConfig.Slot0.kS = 0.048038; // Volts
    motorConfig.Slot0.kV = 18.98; // Volts / (motion profile velocity)
    motorConfig.Slot0.kA = 0.19896; // Volts / (motion profile acceleration)
    motorConfig.Slot0.kG = 0.33558; // Volts / cosine(measured position)
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.Slot0.kP = 277.05; // Volts / (position error)
    motorConfig.Slot0.kD = 6.2346; // Volts / (delta position error)

    motorConfig.Feedback.RotorToSensorRatio = 1.0;
    motorConfig.Feedback.SensorToMechanismRatio = 153.5;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT_MIN.in(Rotations);
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_MAX.in(Rotations);

    // In Mechanism space, so 3 rot/s/s or 1080 deg/s/s
    // This is wicked fast and the motion profile is followed to a T
    motorConfig.MotionMagic.MotionMagicAcceleration = 3;
    // We should never need to change this value - we will never reach this speed.
    // Nonetheless, it must be set so that the max velocity is nonzero.
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 3;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(motorConfig);

    // Detect if we just booted - position is near 0 and pivot isn't moving
    // todo - proper zero procedure, but this covers pivot falling during boot
    if (getPosition().lt(NEAR_ZERO) && abs(getVelocity()).lt(RotationsPerSecond.of(1e-9))) {
      motor.setPosition(PIVOT_OFFSET.in(Rotations));
    }

    setDefaultCommand(aimCommand(Degrees.of(0)));
  }

  public Command aimCommand(Measure<Angle> target) {
    return run(() -> aim(target)).withName("Pivot::Aim");
  }

  public Command manualCommand(Supplier<Double> manualInput) {
    return run(() -> manual(manualInput.get())).withName("Pivot::Manual");
  }

  public void aim(Measure<Angle> target) {
    motor.setControl(positionControl.withPosition(target.plus(PIVOT_OFFSET).in(Rotations)));
  }

  public void manual(double pivotOutput) {
    motor.setControl(manualControl.withOutput(pivotOutput));
  }

  public Measure<Angle> getPosition() {
    return Rotations.of(pitchSignal.refresh().getValue());
  }

  public Measure<Velocity<Angle>> getVelocity() {
    return RotationsPerSecond.of(velocitySignal.refresh().getValue());
  }

  public Measure<Angle> getTargetPosition() {
    return Rotations.of(positionControl.Position);
  }

  public boolean isReady(Measure<Angle> tolerance) {
    RJLog.log("Pivot/tolerance", tolerance.in(Degrees));
    RJLog.log("Pivot/manual", manualControl.Output != 0);
    RJLog.log("Pivot/1", abs(getTargetPosition().minus(getPosition())).in(Degrees));
    RJLog.log("Pivot/2", abs(getTargetPosition().minus(getPosition())).lt(tolerance));

    return manualControl.Output != 0 || abs(getPosition().minus(getTargetPosition())).lt(tolerance);
  }

  @Override
  public void periodic() {
    RJLog.log("Pivot/position", getPosition());
    RJLog.log("Pivot/target", getTargetPosition());
    RJLog.log("Pivot/error", getTargetPosition().minus(getPosition()).in(Degrees));
  }

  public Command sysidPivot() {
    return pivotRoutine
        .quasistatic(Direction.kForward)
        .until(() -> getPosition().gt(PIVOT_MAX.minus(Degrees.of(5))))
        .andThen(motor::stopMotor)
        .andThen(
            pivotRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> getPosition().lt(PIVOT_MIN.plus(Degrees.of(5)))))
        .andThen(motor::stopMotor)
        .andThen(
            pivotRoutine
                .dynamic(Direction.kForward)
                .until(() -> getPosition().gt(PIVOT_MAX.minus(Degrees.of(5)))))
        .andThen(motor::stopMotor)
        .andThen(
            pivotRoutine
                .dynamic(Direction.kReverse)
                .until(() -> getPosition().lt(PIVOT_MIN.plus(Degrees.of(5)))))
        .andThen(motor::stopMotor)
        .withName("Pivot::SysIdPivot");
  }
}
