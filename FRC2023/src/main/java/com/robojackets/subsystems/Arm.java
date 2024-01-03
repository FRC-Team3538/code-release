// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.robojackets.lib.FalconHelper;
import com.robojackets.lib.SignalManager;
import com.robojackets.subsystems.gearbox.FalconGearboxIO;
import com.robojackets.subsystems.gearbox.GearboxIOInputsAutoLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class Arm {
  public DigitalInput sliderLimitSwitch = new DigitalInput(0);
  public CANcoder elbowCoder = new CANcoder(24, "3538_Robot");

  private final StatusSignalValue<Double> encoderAbsolutePositionSignal;

  private final FalconGearboxIO sliderGearbox;
  private final FalconGearboxIO elbowGearbox;

  private final GearboxIOInputsAutoLogged sliderIO = new GearboxIOInputsAutoLogged();
  private final GearboxIOInputsAutoLogged elbowIO = new GearboxIOInputsAutoLogged();

  private final SoftwareLimitSwitchConfigs sliderLimits = new SoftwareLimitSwitchConfigs();
  private final SoftwareLimitSwitchConfigs elbowLimits = new SoftwareLimitSwitchConfigs();

  public Arm() {
    sliderLimits.ForwardSoftLimitEnable = true;
    sliderLimits.ReverseSoftLimitEnable = true;
    sliderLimits.ForwardSoftLimitThreshold = 20;
    sliderLimits.ReverseSoftLimitThreshold = 1;

    elbowLimits.ForwardSoftLimitEnable = true;
    elbowLimits.ReverseSoftLimitEnable = true;
    elbowLimits.ForwardSoftLimitThreshold = 270;
    elbowLimits.ReverseSoftLimitThreshold = -30;

    var sliderMotor = new TalonFX(22, "3538_Robot");

    FalconHelper.applyConfig(
        sliderMotor,
        config -> {
          config.Feedback.SensorToMechanismRatio = 58.1 / 21.0; // rot / in

          config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

          config.Slot0.kP = 9;
          config.Slot0.kD = 1.3;

          config.TorqueCurrent.PeakForwardTorqueCurrent = 200;
          config.TorqueCurrent.PeakReverseTorqueCurrent = -200;

          config.SoftwareLimitSwitch = sliderLimits;
        });

    var elbowMotor = new TalonFX(23, "3538_Robot");

    FalconHelper.applyConfig(
        elbowMotor,
        config -> {
          config.Feedback.SensorToMechanismRatio = 68.08 / 180.0; // rot / deg

          config.TorqueCurrent.PeakForwardTorqueCurrent = 20;
          config.TorqueCurrent.PeakReverseTorqueCurrent = -20;

          config.MotorOutput.PeakForwardDutyCycle = 0.3;
          config.MotorOutput.PeakReverseDutyCycle = -0.3;
          config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

          config.Slot0.kP = 3.1; // An error of 0.5 rotations results in 12 V output
          config.Slot0.kD = 0.5;

          config.SoftwareLimitSwitch = elbowLimits;
        });

    // CANcoderHelper.applyConfig(elbowCoder, config -> {
    //   config.MagnetSensor.MagnetOffset = -0.05;
    // });

    sliderGearbox = new FalconGearboxIO(sliderMotor);
    elbowGearbox = new FalconGearboxIO(elbowMotor);

    encoderAbsolutePositionSignal = elbowCoder.getAbsolutePosition();

    SignalManager.getInstance().register(encoderAbsolutePositionSignal);

    // Elbow Abs Encoder Angle (Startup seed)
    syncElbowWithEncoder();
  }

  public void syncElbowWithEncoder() {
    double absEncoder = encoderAbsolutePositionSignal.getValue() * 360.0;
    elbowGearbox.setPosition(absEncoder);
  }

  public void periodic() {
    sliderGearbox.updateInputs(sliderIO);
    elbowGearbox.updateInputs(elbowIO);

    Logger.getInstance().processInputs("Arm/Slider", sliderIO);
    Logger.getInstance().processInputs("Arm/Elbow", elbowIO);

    SmartDashboard.putBoolean("GoTo/sliderOk", SliderAtSetpoint());
    SmartDashboard.putBoolean("GoTo/ElbowOk", ElbowAtSetpoint());
  }

  public void setSoftLimitsDisable() {
    sliderLimits.ForwardSoftLimitEnable = false;
    sliderLimits.ReverseSoftLimitEnable = false;
    sliderGearbox.getMainMotor().getConfigurator().apply(sliderLimits);

    elbowLimits.ForwardSoftLimitEnable = false;
    elbowLimits.ReverseSoftLimitEnable = false;
    elbowGearbox.getMainMotor().getConfigurator().apply(elbowLimits);
  }

  public void setSoftLimitsEnable() {
    sliderLimits.ForwardSoftLimitEnable = true;
    sliderLimits.ReverseSoftLimitEnable = true;
    sliderGearbox.getMainMotor().getConfigurator().apply(sliderLimits);

    elbowLimits.ForwardSoftLimitEnable = true;
    elbowLimits.ReverseSoftLimitEnable = true;
    elbowGearbox.getMainMotor().getConfigurator().apply(elbowLimits);
  }

  public void SliderSetZero() {
    sliderGearbox.setPosition(0);
  }

  public void ElbowSetZero() {
    elbowGearbox.setPosition(0);
  }

  private PositionTorqueCurrentFOC sliderPositionCmd = new PositionTorqueCurrentFOC(0);
  private PositionTorqueCurrentFOC elbowPositionCmd = new PositionTorqueCurrentFOC(0);

  public void SliderGoTo(double inches) {
    sliderGearbox.setControl(sliderPositionCmd.withPosition(inches));
  }

  public void ElbowGoTo(double degrees) {
    elbowGearbox.setControl(elbowPositionCmd.withPosition(degrees));
  }

  private DutyCycleOut elbowManualCmd = new DutyCycleOut(0);
  private DutyCycleOut sliderManualCmd = new DutyCycleOut(0);

  public void ElbowCmd(double input) {
    double smartdashfun = elbowIO.getPosition();
    //  SmartDashboard.putNumber("ElbowPos2.0", smartdashfun);

    // Gravity Compensation Code:
    double conversionboi = Math.toRadians(smartdashfun);
    double Kf = (0.03 * Math.sin(conversionboi));

    //  SmartDashboard.putNumber("Elbowkf", Kf);

    elbowGearbox.setControl(elbowManualCmd.withOutput(input + Kf));
  }

  public void SliderCmd(double input) {
    sliderGearbox.setControl(sliderManualCmd.withOutput(input));
  }

  public boolean GoTo(double inches, double degrees) {
    SliderGoTo(inches);
    ElbowGoTo(degrees);

    return SliderAtSetpoint() && ElbowAtSetpoint();
  }

  public boolean SliderAtSetpoint() {
    return Math.abs(sliderIO.getPosition() - sliderPositionCmd.Position) < 1.0; // inches
  }

  public boolean ElbowAtSetpoint() {
    return Math.abs(elbowIO.getPosition() - elbowPositionCmd.Position) < 5.0; // degrees
  }

  public boolean AtSetpoint() {
    return SliderAtSetpoint() && ElbowAtSetpoint();
  }

  public boolean GoTo(ArmSetpoint setpoint) {
    SliderGoTo(setpoint.getLength());
    ElbowGoTo(setpoint.getAngle());

    return SliderAtSetpoint() && ElbowAtSetpoint();
  }

  public double GetSliderPosition() {
    return sliderIO.getPosition();
  }

  public double getElbowPosition() {
    return elbowIO.getPosition();
  }

  public boolean JogElbow(double offset) {
    ElbowGoTo(elbowIO.getPosition() + offset);

    return ElbowAtSetpoint();
  }

  public boolean JogSlider(double offset) {
    SliderGoTo(sliderIO.getPosition() + offset);

    return SliderAtSetpoint();
  }
}
