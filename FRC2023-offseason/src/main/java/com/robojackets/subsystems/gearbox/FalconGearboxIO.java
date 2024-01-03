package com.robojackets.subsystems.gearbox;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.robojackets.lib.FalconHelper;
import com.robojackets.lib.SignalManager;
import java.util.List;
import java.util.function.Consumer;

public class FalconGearboxIO extends GearboxIO {
  private final TalonFX main;
  private final TalonFX[] followers;

  private final StatusSignal<Double> supplyVoltageSignal;
  private final StatusSignal<Double> dutyCycleSignal;
  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> supplyCurrentSignal;
  private final StatusSignal<Double> statorCurrentSignal;
  private final StatusSignal<Double> temperatureSignal;
  private final StatusSignal<Double> torqueCurrentSignal;
  private final StatusSignal<Double> closedLoopErrorSignal;

  public FalconGearboxIO(TalonFX main, TalonFX... followers) {
    this.main = main;
    this.followers = followers;

    supplyVoltageSignal = main.getSupplyVoltage();
    dutyCycleSignal = main.getDutyCycle();
    positionSignal = main.getPosition();
    velocitySignal = main.getVelocity();
    supplyCurrentSignal = main.getSupplyCurrent();
    statorCurrentSignal = main.getTorqueCurrent();
    temperatureSignal = main.getDeviceTemp();
    torqueCurrentSignal = main.getTorqueCurrent();

    closedLoopErrorSignal = main.getClosedLoopError();

    for (var motor : followers) {
      motor.setControl(new Follower(main.getDeviceID(), false));
    }

    SignalManager.getInstance()
        .register(
            supplyVoltageSignal,
            dutyCycleSignal,
            positionSignal,
            velocitySignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            temperatureSignal,
            torqueCurrentSignal,
            closedLoopErrorSignal);
  }

  public List<StatusSignal<Double>> getCriticalSignals() {
    return List.of(positionSignal, velocitySignal);
  }

  @Override
  public boolean isOnCanivore() {
    return !main.getNetwork().equals("rio");
  }

  @Override
  public void setInverted(boolean inverted) {
    forEach(
        motor -> {
          FalconHelper.updateMotorOutputConfigs(
              motor,
              config ->
                  config.Inverted =
                      inverted
                          ? InvertedValue.Clockwise_Positive
                          : InvertedValue.CounterClockwise_Positive);
        });
  }

  @Override
  public void setCurrentLimit(double amps, boolean enabled) {
    forEach(
        motor -> {
          FalconHelper.updateCurrentLimitsConfigs(
              motor,
              config -> {
                config.SupplyCurrentLimit = amps;
                config.SupplyCurrentLimitEnable = enabled;
              });
        });
  }

  @Override
  public void setBrakeMode(boolean brake) {
    forEach(
        motor -> {
          FalconHelper.updateMotorOutputConfigs(
              motor,
              config ->
                  config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
  }

  @Override
  public StatusCode setControl(ControlRequest request) {
    return main.setControl(request);
  }

  @Override
  public void updateInputs(GearboxIOInputs ioInputs) {
    ioInputs.voltage = supplyVoltageSignal.getValue() * dutyCycleSignal.getValue();
    ioInputs.dutyCycle = dutyCycleSignal.getValue();

    ioInputs.position = positionSignal.getValue();
    ioInputs.velocity = velocitySignal.getValue();

    ioInputs.supplyCurrentAmps = supplyCurrentSignal.getValue();
    ioInputs.statorCurrentAmps = statorCurrentSignal.getValue();
    ioInputs.temperatureCelsius = temperatureSignal.getValue();

    ioInputs.torqueCurrentAmps = torqueCurrentSignal.getValue();
    ioInputs.closedLoopError = closedLoopErrorSignal.getValue();
  }

  public void forEach(Consumer<TalonFX> func) {
    func.accept(main);
    for (TalonFX talonFX : followers) {
      func.accept(talonFX);
    }
  }

  public TalonFX getMainMotor() {
    return main;
  }

  @Override
  public StatusCode setPosition(double position) {
    return main.setPosition(position);
  }
}
