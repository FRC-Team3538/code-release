package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX roller = new TalonFX(17, "*");
  private final TalonFX wheels = new TalonFX(18, "*");

  private final DutyCycleOut control = new DutyCycleOut(0);

  public Intake() {
    Preferences.initDouble("Intake/Output", 1);

    var rollerConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    roller.getConfigurator().apply(rollerConfig);

    var wheelsConfig = new TalonFXConfiguration();

    wheelsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    wheels.getConfigurator().apply(wheelsConfig);

    setDefaultCommand(idleCommand());
  }

  public Command intakeCommand() {
    return run(this::intake).withName("Intake::Intake");
  }

  public Command rejectCommand() {
    return run(this::reject).withName("Intake::Reject");
  }

  public Command idleCommand() {
    return run(this::idle).withName("Intake::Idle");
  }

  public void intake() {
    var output = Preferences.getDouble("Intake/Output", 1);

    roller.setControl(control.withOutput(output));
    wheels.setControl(control.withOutput(output));
  }

  public void reject() {
    var output = Preferences.getDouble("Intake/Output", 1);

    roller.setControl(control.withOutput(-output));
    wheels.setControl(control.withOutput(-output));
  }

  public void idle() {
    roller.setControl(control.withOutput(0));
    wheels.setControl(control.withOutput(0));
  }
}
