package com.robojackets.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter {
  private TalonFX shooter1 = new TalonFX(18, "*");
  private TalonFX shooter2 = new TalonFX(19, "*");
  private TalonFX feeder = new TalonFX(20, "*");
  private DigitalInput beambreak = new DigitalInput(2);

  private Alliance alliance = Alliance.Blue;

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  private InterpolatingDoubleTreeMap interpolateRPM = new InterpolatingDoubleTreeMap();

  VelocityVoltage shooter1Control = new VelocityVoltage(0).withSlot(0);
  VelocityVoltage shooter2Control = new VelocityVoltage(0).withSlot(0);
  DutyCycleOut feederControl = new DutyCycleOut(0);
  TorqueCurrentFOC feedercurrent = new TorqueCurrentFOC(0);
  TorqueCurrentFOC shooter1Current = new TorqueCurrentFOC(0);
  TorqueCurrentFOC shooter2Current = new TorqueCurrentFOC(0);

  VelocityTorqueCurrentFOC shooter1torqueControl = new VelocityTorqueCurrentFOC(0).withSlot(1);
  VelocityTorqueCurrentFOC shooter2torqueControl = new VelocityTorqueCurrentFOC(0).withSlot(1);

  MotionMagicVelocityTorqueCurrentFOC shooter1motionmagic =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(1);
  MotionMagicVelocityTorqueCurrentFOC shooter2motionmagic =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(1);

  // Values for motion magic is 180 and 100
  // boolean m_shooter1OK = false;
  // boolean m_shooter2OK = false;

  SysIdRoutine shooterRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second),
              Volts.of(50),
              Seconds.of(10),
              state -> SignalLogger.writeString("shooter-sysid-state", state.toString())),
          new SysIdRoutine.Mechanism(
              amps -> {
                shooter1.setControl(shooter1Current.withOutput(amps.in(Volts)));
                shooter2.setControl(shooter2Current.withOutput(amps.in(Volts)));
              },
              null,
              new Subsystem() {},
              "shooter"));

  private double cmd_shooterrpm = 0.0;

  private NetworkTableEntry spinScalarEntry =
      NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SPIN_SCALAR");

  private static double DEFAULT_SPIN_RATIO = 1.3;

  public Shooter() {
    var FeederTalonFXConfigs = new TalonFXConfiguration();
    var ShooterTalonFXConfigs = new TalonFXConfiguration();

    ShooterTalonFXConfigs.Slot0.kP = 0.5;
    ShooterTalonFXConfigs.Slot0.kS = 0.4;
    ShooterTalonFXConfigs.Slot0.kV = 0.12;
    ShooterTalonFXConfigs.Slot0.kI = 0.02;
    // ShooterTalonFXConfigs.Slot0.kA = 0.011957;

    ShooterTalonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 100;
    ShooterTalonFXConfigs.MotionMagic.MotionMagicAcceleration = 120;

    ShooterTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooter1.getConfigurator().apply(ShooterTalonFXConfigs);

    ShooterTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooter2.getConfigurator().apply(ShooterTalonFXConfigs);

    Slot1Configs shooter1Configs =
        new Slot1Configs().withKS(14.32).withKV(0.14).withKA(0.7).withKP(12.0);
    shooter1.getConfigurator().apply(shooter1Configs);

    Slot1Configs shooter2Configs =
        new Slot1Configs().withKS(18.683).withKV(0.3).withKA(0.74395).withKP(12.0);
    shooter2.getConfigurator().apply(shooter2Configs);

    feeder.getConfigurator().apply(FeederTalonFXConfigs);

    // interpolateRPM.put(1.18, 3000.0);
    // interpolateRPM.put(1.79, 3000.0);
    // interpolateRPM.put(2.4, 3000.0);
    // interpolateRPM.put(3.06, 3000.0);
    // interpolateRPM.put(3.69, 3000.0);
    // interpolateRPM.put(4.23, 4000.0);
    // interpolateRPM.put(4.8, 4000.0);

    interpolateRPM.put(1.18, 3000.0);
    interpolateRPM.put(2.4, 3000.0);
    interpolateRPM.put(3.69, 3000.0);
    interpolateRPM.put(3.89, 3100.0);
    interpolateRPM.put(4.23, 3100.0);
    interpolateRPM.put(4.5, 3200.0);

    // interpolateRPM.put(4.8, 4000.0);

    spinScalarEntry.setDouble(DEFAULT_SPIN_RATIO);
  }

  public double getInterpolateVals(double distance) {
    // SmartDashboard.putNumber("interpolate rpm val", interpolateRPM.get(distance));
    return interpolateRPM.get(distance);
  }

  public void OverrideFeeder(double dutycycle) {
    feeder.setControl(feederControl.withOutput(dutycycle));
  }

  public void setFeeder(double dutycycle, boolean ignoreBeamBreak) {
    if (beambreak.get() == true || ignoreBeamBreak) {
      feeder.setControl(feederControl.withOutput(dutycycle));
    } else {
      feeder.setControl(feederControl.withOutput(0.0));
    }
  }

  // TODO swap spin ratio left/right based on red/blue. blue -> shooter1, red -> shooter2
  public boolean AtRPM(double tolerance) {
    boolean m_shooter1OK, m_shooter2OK;

    if (Alliance.Red.equals(alliance)) {
      m_shooter1OK =
          Math.abs((shooter1.getVelocity().getValue() * 60.0) - cmd_shooterrpm) < tolerance;
      m_shooter2OK =
          Math.abs(
                  (shooter2.getVelocity().getValue() * 60.0)
                      - cmd_shooterrpm * spinScalarEntry.getDouble(DEFAULT_SPIN_RATIO))
              < tolerance;
    } else {
      m_shooter1OK =
          Math.abs(
                  (shooter1.getVelocity().getValue() * 60.0)
                      - cmd_shooterrpm * spinScalarEntry.getDouble(DEFAULT_SPIN_RATIO))
              < tolerance;
      m_shooter2OK =
          Math.abs((shooter2.getVelocity().getValue() * 60.0) - cmd_shooterrpm) < tolerance;
    }

    SmartDashboard.putBoolean("shooter1ok", m_shooter1OK);
    SmartDashboard.putBoolean("shooter2ok", m_shooter2OK);

    return m_shooter1OK && m_shooter2OK;
  }

  public void SetShooterRPM(double rpm) {
    cmd_shooterrpm = rpm;

    if (Alliance.Red.equals(alliance)) {
      shooter1.setControl(shooter1motionmagic.withVelocity(cmd_shooterrpm / 60.0));
      shooter2.setControl(
          shooter2motionmagic.withVelocity(
              cmd_shooterrpm / 60.0 * spinScalarEntry.getDouble(DEFAULT_SPIN_RATIO)));
    } else {
      shooter1.setControl(
          shooter1motionmagic.withVelocity(
              cmd_shooterrpm / 60.0 * spinScalarEntry.getDouble(DEFAULT_SPIN_RATIO)));
      shooter2.setControl(shooter2motionmagic.withVelocity(cmd_shooterrpm / 60.0));
    }
  }

  public boolean NoteNotDetected() {
    return beambreak.get();
  }

  public boolean NoteDetected() {
    return !beambreak.get();
  }

  public double getShooterRPM() {
    return cmd_shooterrpm;
  }

  public void SDOutputs() {
    SmartDashboard.putNumber("shooter1rpm", shooter1.getVelocity().getValue() * 60.0);
    SmartDashboard.putNumber("shooter2rpm", shooter2.getVelocity().getValue() * 60.0);
    SmartDashboard.putBoolean("In Feeder?", !beambreak.get());

    SmartDashboard.putBoolean("shootersok?", AtRPM(150));
  }

  public Command sysidShooter() {
    return shooterRoutine
        .quasistatic(Direction.kForward)
        .until(
            () ->
                Math.abs(shooter1.getVelocity().getValue()) > 50
                    || Math.abs(shooter2.getVelocity().getValue()) > 50)
        .andThen(
            Commands.run(
                    () -> {
                      shooter1.setControl(shooter1Current.withOutput(0));
                      shooter2.setControl(shooter2Current.withOutput(0));
                    })
                .until(() -> Math.abs(shooter1.getVelocity().getValue()) < 0.1))
        .andThen(
            shooterRoutine
                .quasistatic(Direction.kReverse)
                .until(
                    () ->
                        Math.abs(shooter1.getVelocity().getValue()) > 50
                            || Math.abs(shooter2.getVelocity().getValue()) > 50))
        .andThen(
            Commands.run(
                    () -> {
                      shooter1.setControl(shooter1Current.withOutput(0));
                      shooter2.setControl(shooter2Current.withOutput(0));
                    })
                .until(() -> Math.abs(shooter1.getVelocity().getValue()) < 0.1))
        .andThen(
            shooterRoutine
                .dynamic(Direction.kForward)
                .until(
                    () ->
                        Math.abs(shooter1.getVelocity().getValue()) > 50
                            || Math.abs(shooter2.getVelocity().getValue()) > 50))
        .andThen(
            Commands.run(
                    () -> {
                      shooter1.setControl(shooter1Current.withOutput(0));
                      shooter2.setControl(shooter2Current.withOutput(0));
                    })
                .until(() -> Math.abs(shooter1.getVelocity().getValue()) < 0.1))
        .andThen(
            shooterRoutine
                .dynamic(Direction.kReverse)
                .until(
                    () ->
                        Math.abs(shooter1.getVelocity().getValue()) > 50
                            || Math.abs(shooter2.getVelocity().getValue()) > 50))
        .andThen(
            Commands.run(
                    () -> {
                      shooter1.setControl(shooter1Current.withOutput(0));
                      shooter2.setControl(shooter2Current.withOutput(0));
                    })
                .until(() -> Math.abs(shooter1.getVelocity().getValue()) < 0.1))
        .withName("Shooter::SysIdShooter");
  }
}
