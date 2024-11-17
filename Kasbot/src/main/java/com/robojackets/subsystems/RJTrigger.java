package com.robojackets.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.robojackets.lib.RJLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RJTrigger extends SubsystemBase {
  private final TalonFX motor = new TalonFX(23, "*");
  private DigitalInput beambreak = new DigitalInput(4);

  DutyCycleOut control = new DutyCycleOut(0);

  public RJTrigger() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);

    Preferences.initDouble("Trigger/PrimeOutput", 1);
    Preferences.initDouble("Trigger/FireOutput", 1);
    Preferences.initDouble("Trigger/FireSustain", 0.25); // seconds

    setDefaultCommand(idleCommand());
  }

  @Override
  public void periodic() {
    RJLog.log("Trigger/Note", notePresent());
  }

  public Command fireCommand() {
    var fireSustain = Preferences.getDouble("Trigger/FireSustain", 0.25); // seconds

    return run(this::fire)
        .until(this::noteAbsent)
        .andThen(waitSeconds(fireSustain))
        .withName("RJTrigger::Fire");
  }

  public Command primeCommand() {
    return run(this::prime).until(this::notePresent).withName("RJTrigger::Prime");
  }

  public Command idleCommand() {
    return run(this::idle).withName("RJTrigger::Idle");
  }

  public Command poopCommand() {
    return run(this::poop).withName("RJTrigger::Poop");
  }

  public void fire() {
    var output = Preferences.getDouble("Trigger/FireOutput", 1);
    motor.setControl(control.withOutput(output));
  }

  public void prime() {
    var output = Preferences.getDouble("Trigger/PrimeOutput", 1);
    motor.setControl(control.withOutput(output));
  }

  public void idle() {
    motor.setControl(control.withOutput(0));
  }

  public void poop() {
    var output = Preferences.getDouble("Trigger/Output", 1);
    motor.setControl(control.withOutput(-output));
  }

  public boolean notePresent() {
    return !beambreak.get();
  }

  public boolean noteAbsent() {
    return !notePresent();
  }
}
