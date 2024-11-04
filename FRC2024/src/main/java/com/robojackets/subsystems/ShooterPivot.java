package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterPivot {
  private TalonFX shooterpivot = new TalonFX(21, "*");
  private CANcoder shootercancoder = new CANcoder(30, "*");

  private DutyCycleOut request = new DutyCycleOut(0);
  private PositionVoltage positionControl = new PositionVoltage(0);
  private MotionMagicVoltage voltageControl = new MotionMagicVoltage(0);

  private InterpolatingDoubleTreeMap interpolatePitch = new InterpolatingDoubleTreeMap();

  private double cmd_angle = 0.0;

  private final SoftwareLimitSwitchConfigs pivotlimits = new SoftwareLimitSwitchConfigs();

  public ShooterPivot() {

    // interpolatePitch.put(1.18, 34.01);
    // interpolatePitch.put(1.79, 25.04);
    // interpolatePitch.put(2.4, 17.13);
    // interpolatePitch.put(3.06, 12.04);
    // interpolatePitch.put(3.69, 9.32);
    // interpolatePitch.put(4.23, 6.59 - 0.65);
    // interpolatePitch.put(4.8, 5.55 - 1.75);

    interpolatePitch.put(1.18, 37.65);
    interpolatePitch.put(2.4, 21.3);
    interpolatePitch.put(3.69, 12.125);
    interpolatePitch.put(3.89, 10.8);
    interpolatePitch.put(4.23, 9.6);
    interpolatePitch.put(4.5, 8.075);
    // interpolatePitch.put(4.8, 5.95);
    // interpolator.get

    var PivotTalonFXConfigs = new TalonFXConfiguration();
    var PivotCANcoderConfigs = new CANcoderConfiguration();

    PivotTalonFXConfigs.Slot0.kP = 900; // 300 KP and 5.0 KI
    // PivotTalonFXConfigs.Slot0.kP = 56;

    PivotTalonFXConfigs.Feedback.RotorToSensorRatio = 1.0; // 139.2;
    PivotTalonFXConfigs.Feedback.SensorToMechanismRatio = 139.2; // 1.0;
    PivotTalonFXConfigs.Feedback.FeedbackRemoteSensorID = 30;
    PivotTalonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    PivotTalonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    PivotTalonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    PivotTalonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.8 / 360.0;
    PivotTalonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 45 / 360.0;

    PivotTalonFXConfigs.MotionMagic.MotionMagicAcceleration = 2;
    PivotTalonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 2;

    PivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PivotCANcoderConfigs.MagnetSensor.MagnetOffset = 0.427490;

    shooterpivot.getConfigurator().apply(PivotTalonFXConfigs);
    shootercancoder.getConfigurator().apply(PivotCANcoderConfigs);

    shooterpivot.setPosition(0);
  }

  public boolean AtAngle(double tolerance) {
    boolean shooterpivotOk =
        Math.abs((shooterpivot.getPosition().getValue() * 360.0) - cmd_angle) < tolerance;

    SmartDashboard.putBoolean("shooterpivotok", shooterpivotOk);

    return shooterpivotOk;
  }

  public void setZero() {
    shooterpivot.setPosition(0);
  }

  public void setAngle(double angle) {
    cmd_angle = angle;

    shooterpivot.setControl(voltageControl.withPosition(cmd_angle / 360.0));
  }

  public double getAngle() {
    return shooterpivot.getPosition().getValueAsDouble() * 360.0;
  }

  public double getAngleCommanded() {
    return cmd_angle;
  }

  public void manual(double dutycycle) {
    shooterpivot.setControl(request.withOutput(-dutycycle));
  }

  public void setSoftLimitsDisable() {
    pivotlimits.ForwardSoftLimitEnable = false;
    pivotlimits.ReverseSoftLimitEnable = false;

    shooterpivot.getConfigurator().apply(pivotlimits);
  }

  public void setSoftLimitsEnable() {
    pivotlimits.ForwardSoftLimitEnable = true;
    pivotlimits.ReverseSoftLimitEnable = true;
    pivotlimits.ReverseSoftLimitThreshold = 0.0;
    pivotlimits.ForwardSoftLimitThreshold = 45 / 360.0;

    shooterpivot.getConfigurator().apply(pivotlimits);
  }

  public void PivotReadout() {
    SmartDashboard.putNumber("pivot in degrees", shooterpivot.getPosition().getValue() * 360.0);
    SmartDashboard.putBoolean("shooterpivotok", AtAngle(0.6));
  }

  public void setCoastMode() {
    shooterpivot.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getInterpolateVals(double distance) {
    // SmartDashboard.putNumber("interpolate pitch val", interpolatePitch.get(distance));
    return interpolatePitch.get(distance);
  }
}
