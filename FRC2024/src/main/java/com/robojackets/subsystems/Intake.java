package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
  private TalonFX intakeMotor = new TalonFX(17, "*");

  private DutyCycleOut request = new DutyCycleOut(0);

  public Intake() {
    var IntakeTalonFXConfigs = new TalonFXConfiguration();
    intakeMotor.getConfigurator().apply(IntakeTalonFXConfigs);
  }

  public void run(double value) {
    // intakeMotor.setControl(request.withOutput(value));
    intakeMotor.setControl(request.withOutput(value));
  }

  public double getSupplyCurrent() {
    return intakeMotor.getSupplyCurrent().getValue();
  }

  public double getStatorCurrent() {
    return intakeMotor.getStatorCurrent().getValue();
  }

  public void IntakeSDOutputs() {
    SmartDashboard.putNumber("intake supply current", getSupplyCurrent());
    SmartDashboard.putNumber("intake stator current", getStatorCurrent());
  }
}
