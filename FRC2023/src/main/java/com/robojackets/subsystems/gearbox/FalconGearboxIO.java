package com.robojackets.subsystems.gearbox;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.robojackets.lib.FalconHelper;
import com.robojackets.lib.SignalManager;
import java.util.List;
import java.util.function.Consumer;

public class FalconGearboxIO extends GearboxIO {
  private final TalonFX main;
  private final TalonFX[] followers;

  private final StatusSignalValue<Double> supplyVoltageSignal;
  private final StatusSignalValue<Double> dutyCycleSignal;
  private final StatusSignalValue<Double> positionSignal;
  private final StatusSignalValue<Double> velocitySignal;
  private final StatusSignalValue<Double> supplyCurrentSignal;
  private final StatusSignalValue<Double> statorCurrentSignal;
  private final StatusSignalValue<Double> temperatureSignal;
  private final StatusSignalValue<Double> torqueCurrentSignal;
  private final StatusSignalValue<Double> closedLoopErrorSignal;

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

  public List<StatusSignalValue<Double>> getCriticalSignals() {
    return List.of(positionSignal, velocitySignal);
  }

  @Override
  public boolean isOnCanivore() {
    return !main.getCANBus().equals("rio");
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
    return main.setRotorPosition(position);
  }
}
