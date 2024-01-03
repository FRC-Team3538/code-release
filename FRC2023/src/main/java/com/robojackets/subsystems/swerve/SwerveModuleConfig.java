package com.robojackets.subsystems.swerve;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.robojackets.lib.CANcoderHelper;
import com.robojackets.lib.FalconHelper;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class SwerveModuleConfig {
  private SimpleMotorFeedforward driveFeedforward;
  private SimpleMotorFeedforward azimuthFeedforward;

  private String canBus;

  private double driveVelocityPIDkP;
  private double azimuthPositionPIDkP;
  private double azimuthPositionPIDkD;
  private double azimuthMotionMagicCruiseVelocity;
  private double azimuthMotionMagicAcceleration;

  public FalconSwerveModuleIO applyWithOffset(
      FalconSwerveModuleIO moduleIO, Rotation2d offset, int encoderID) {
    return moduleIO
        .configureDrive(
            motor -> {
              FalconHelper.applyConfig(
                  motor,
                  config -> {
                    config.Slot0.kP = driveVelocityPIDkP;
                    config.Slot0.kI = 0;
                    config.Slot0.kD = 0;
                    config.Slot0.kS = 0; // driveFeedforward.ks;
                    config.Slot0.kV = 0; // driveFeedforward.kv;

                    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                    config.CurrentLimits.SupplyCurrentLimit = 60;
                    config.CurrentLimits.SupplyCurrentLimitEnable = true;
                  });
            })
        .configureAzimuth(
            motor -> {
              FalconHelper.applyConfig(
                  motor,
                  config -> {
                    config.Slot0.kP = azimuthPositionPIDkP;
                    config.Slot0.kI = 0;
                    config.Slot0.kD = azimuthPositionPIDkD;
                    config.Slot0.kS = 0;
                    config.Slot0.kV = 0;

                    config.ClosedLoopGeneral.ContinuousWrap = true;

                    config.Feedback.RotorToSensorRatio = 12.8;
                    config.Feedback.SensorToMechanismRatio = 1.0;
                    config.Feedback.FeedbackRemoteSensorID = encoderID;
                    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

                    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    config.MotorOutput.DutyCycleNeutralDeadband = 0.01;

                    config.CurrentLimits.SupplyCurrentLimit = 30;
                    config.CurrentLimits.SupplyCurrentThreshold = 60;
                    config.CurrentLimits.SupplyTimeThreshold = 0.25;
                    config.CurrentLimits.SupplyCurrentLimitEnable = true;
                  });
            })
        .configureEncoder(
            encoder -> {
              CANcoderHelper.updateMagnetSensorConfigs(
                  encoder,
                  config -> {
                    config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                    config.MagnetOffset = offset.getRotations();
                  });
            });
  }
}
