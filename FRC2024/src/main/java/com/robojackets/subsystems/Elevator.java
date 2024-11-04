package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

  TalonFX elevatorMotor = new TalonFX(22, "*");

  public enum ElevatorStates {
    MANUAL,
    POSITION
  }

  private ElevatorStates m_elevstates;

  private double cmd_elevatorpos = 0.0;

  // PositionVoltage elevatorControl = new PositionVoltage(0);
  MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0);
  DutyCycleOut manualControl = new DutyCycleOut(0);

  private final SoftwareLimitSwitchConfigs elevatorlimits = new SoftwareLimitSwitchConfigs();

  public Elevator() {
    // elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    var ElevatorTalonFXConfigs = new TalonFXConfiguration();

    ElevatorTalonFXConfigs.Slot0.kP = 5;
    ElevatorTalonFXConfigs.Slot0.kV = 0.175;

    ElevatorTalonFXConfigs.Feedback.RotorToSensorRatio = 1;
    ElevatorTalonFXConfigs.Feedback.SensorToMechanismRatio = 1.1;

    ElevatorTalonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    ElevatorTalonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    ElevatorTalonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1;
    ElevatorTalonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 17.25;

    ElevatorTalonFXConfigs.MotionMagic.MotionMagicAcceleration = 260;
    ElevatorTalonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 200;

    ElevatorTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorMotor.getConfigurator().apply(ElevatorTalonFXConfigs);
  }

  public void setManualMode() {
    m_elevstates = ElevatorStates.MANUAL;
  }

  public void setPositionMode() {
    m_elevstates = ElevatorStates.POSITION;
  }

  public void ManualCommand(double dutycycle) {
    elevatorMotor.setControl(manualControl.withOutput(-dutycycle));
  }

  public void PositionCommand(double position) {
    cmd_elevatorpos = position;
    elevatorMotor.setControl(elevatorControl.withPosition(cmd_elevatorpos));
  }

  public double getCurrentPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  // public void AtPosition() {
  //   elevatorMotor.
  // }

  public void SetZero() {
    elevatorMotor.setPosition(0.0);
  }

  public void setSoftLimitsDisable() {
    elevatorlimits.ForwardSoftLimitEnable = false;
    elevatorlimits.ReverseSoftLimitEnable = false;

    elevatorMotor.getConfigurator().apply(elevatorlimits);
  }

  public void setSoftLimitsEnable() {
    elevatorlimits.ForwardSoftLimitEnable = true;
    elevatorlimits.ReverseSoftLimitEnable = true;
    elevatorlimits.ForwardSoftLimitThreshold = 17.25;
    elevatorlimits.ReverseSoftLimitThreshold = 1;

    elevatorMotor.getConfigurator().apply(elevatorlimits);
  }

  public void setCoastMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ElevatorReadout() {
    SmartDashboard.putNumber("elevator pos readout", elevatorMotor.getPosition().getValue());
  }

  public double ElevatorVelocity() {
    return elevatorMotor.getVelocity().getValue();
  }
}
