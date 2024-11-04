package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {

  TalonFX climberMotor = new TalonFX(23, "*");

  DutyCycleOut climberControl = new DutyCycleOut(0);

  PositionVoltage holdPositionControl = new PositionVoltage(0);
  private boolean holdPosition = false;

  public Climber() {

    var ClimberTalonFXConfigs = new TalonFXConfiguration();

    ClimberTalonFXConfigs.Slot0.kP = 1;

    ClimberTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // climberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberMotor.getConfigurator().apply(ClimberTalonFXConfigs);
    // ClimberTalonFXConfigs.CurrentLimits.SupplyCurrentLimit
  }

  public void ClimberMove(double dutycycle) {
    climberMotor.setControl(climberControl.withOutput(dutycycle));
    holdPosition = false;
  }

  public void HoldPosition() {
    if (!holdPosition) {
      holdPositionControl.Position = climberMotor.getPosition().getValue();
      holdPosition = true;
    }

    climberMotor.setControl(holdPositionControl);
  }

  public void setCoastMode() {
    climberMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ClimberReadout() {
    SmartDashboard.putNumber("climber readout", climberMotor.getDutyCycle().getValue());
  }
}
