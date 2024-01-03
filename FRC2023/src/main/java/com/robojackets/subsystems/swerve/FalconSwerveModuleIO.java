package com.robojackets.subsystems.swerve;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.robojackets.lib.CTREHelper;
import com.robojackets.lib.SignalManager;
import com.robojackets.lib.UnitConversion;
import com.robojackets.sim.SwerveWheelSim;
import com.robojackets.sim.TurretSim;
import com.robojackets.subsystems.gearbox.FalconGearboxIO;
import com.robojackets.subsystems.gearbox.GearboxIO.GearboxIOInputs;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import java.util.function.Consumer;

public class FalconSwerveModuleIO extends SwerveModuleIO {
  private static final double WHEEL_RADIUS_METERS = 2 * UnitConversion.METERS_PER_INCH;
  private static final double TURN_GEARBOX_RATIO = 12.8;
  private static final double TURN_SCALE_FACTOR =
      UnitConversion.RADIANS_PER_ROTATION / TURN_GEARBOX_RATIO;
  private static final double DRIVE_GEARBOX_RATIO = 6.75;
  private static final double DRIVE_SCALE_FACTOR =
      WHEEL_RADIUS_METERS * UnitConversion.RADIANS_PER_ROTATION / DRIVE_GEARBOX_RATIO;

  private final CANcoder azimuthEncoder;

  private final StatusSignalValue<Double> encoderPositionSignal;

  private final VelocityVoltage driveControlRequest;
  private final PositionVoltage azimuthControlRequest;

  protected final FalconGearboxIO drive;
  protected final FalconGearboxIO azimuth;

  private final SimpleMotorFeedforward driveFeedforward;
  private final SimpleMotorFeedforward azimuthFeedforward;

  private final SwerveWheelSim m_driveSim;
  private final TurretSim m_azimuthSim;

  public FalconSwerveModuleIO(
      TalonFX drive,
      TalonFX azimuth,
      CANcoder azimuthEncoder,
      SimpleMotorFeedforward driveFeedforward,
      SimpleMotorFeedforward azimuthFeedforward) {
    this.drive = new FalconGearboxIO(drive);
    this.azimuth = new FalconGearboxIO(azimuth);
    this.azimuthEncoder = azimuthEncoder;

    this.driveFeedforward = driveFeedforward;
    this.azimuthFeedforward = azimuthFeedforward;

    encoderPositionSignal = this.azimuthEncoder.getPosition();

    driveControlRequest =
        new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withOverrideBrakeDurNeutral(false);
    azimuthControlRequest =
        new PositionVoltage(0).withEnableFOC(true).withSlot(0).withOverrideBrakeDurNeutral(false);

    azimuthControlRequest.UpdateFreqHz = 500;

    m_driveSim =
        new SwerveWheelSim(
            LinearSystemId.identifyPositionSystem(driveFeedforward.kv, driveFeedforward.ka),
            DCMotor.getFalcon500(1),
            6.75,
            2 * UnitConversion.METERS_PER_INCH,
            VecBuilder.fill(0));
    m_azimuthSim =
        new TurretSim(
            LinearSystemId.identifyPositionSystem(azimuthFeedforward.kv, azimuthFeedforward.ka),
            DCMotor.getFalcon500(1),
            12.8,
            VecBuilder.fill(0));

    SignalManager.getInstance().register(encoderPositionSignal);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    Drive(desiredState.speedMetersPerSecond); // setDesiredState uses these drive and turn shi
    Turn(desiredState.angle.getRadians());
  }

  private void Drive(double velocityMetersPerSecond) {
    double ff = driveFeedforward.calculate(velocityMetersPerSecond);
    drive.setControl(
        driveControlRequest
            .withVelocity(velocityMetersPerSecond / DRIVE_SCALE_FACTOR)
            .withFeedForward(ff));
  }

  private void Turn(double targetAngleRadians) {
    var current = Rotation2d.fromRotations(encoderPositionSignal.getValue());
    var target = Rotation2d.fromRadians(targetAngleRadians);
    var error = target.minus(current);

    azimuth.setControl(
        azimuthControlRequest.withPosition(
            targetAngleRadians / UnitConversion.RADIANS_PER_ROTATION));
  }

  @Override
  public void updateInputs(
      SwerveModuleIOInputs ioInputs, GearboxIOInputs driveInputs, GearboxIOInputs azimuthInputs) {
    drive.updateInputs(driveInputs);
    azimuth.updateInputs(azimuthInputs);

    ioInputs.angleRadians = azimuthInputs.getPosition() * UnitConversion.RADIANS_PER_ROTATION;
    ioInputs.distanceMeters = DRIVE_SCALE_FACTOR * driveInputs.getPosition();
    ioInputs.velocityMetersPerSecond = DRIVE_SCALE_FACTOR * driveInputs.getVelocity();
    ioInputs.externalAngleRadians =
        encoderPositionSignal.getValue() * UnitConversion.RADIANS_PER_ROTATION;
  }

  @Override
  public void Stop() {
    drive.setControl(new NeutralOut());
    azimuth.setControl(new NeutralOut());
  }

  public FalconSwerveModuleIO configureDrive(Consumer<TalonFX> change) {
    drive.forEach(change);

    return this;
  }

  public FalconSwerveModuleIO configureAzimuth(Consumer<TalonFX> change) {
    azimuth.forEach(change);

    return this;
  }

  public FalconSwerveModuleIO configureEncoder(Consumer<CANcoder> change) {
    change.accept(azimuthEncoder);

    return this;
  }

  @Override
  public void SimPeriodic() {
    SimDrive();
    SimAzimuth();
  }

  private void SimDrive() {
    var motorSim = drive.getMainMotor().getSimState();

    CTREHelper.checkStatus(motorSim.setSupplyVoltage(12), "driveSim::setSupplyVoltage");
    m_driveSim.setInputVoltage(motorSim.getMotorVoltage());

    for (int i = 0; i < 20; i++) {
      m_driveSim.update(0.001);
    }

    var motor_velocity = m_driveSim.getVelocityMetersPerSecond() / DRIVE_SCALE_FACTOR;
    var motor_position = m_driveSim.getPositionMeters() / DRIVE_SCALE_FACTOR;

    CTREHelper.checkStatus(motorSim.setRotorVelocity(motor_velocity), "driveSim::setRotorVelocity");
    CTREHelper.checkStatus(
        motorSim.setRawRotorPosition(motor_position), "driveSim::setRawRotorPosition");
  }

  private void SimAzimuth() {
    var motorSim = azimuth.getMainMotor().getSimState();
    var encoderSim = azimuthEncoder.getSimState();

    CTREHelper.checkStatus(motorSim.setSupplyVoltage(12), "azimuthSim::setSupplyVoltage");
    CTREHelper.checkStatus(encoderSim.setSupplyVoltage(12), "encoderSim::setSupplyVoltage");
    m_azimuthSim.setInputVoltage(motorSim.getMotorVoltage());

    for (int i = 0; i < 20; i++) {
      m_azimuthSim.update(0.001);
    }

    var motor_velocity = m_azimuthSim.getAngularVelocityRadiansPerSecond() / TURN_SCALE_FACTOR;
    var motor_position = m_azimuthSim.getAngleRadians() / TURN_SCALE_FACTOR;

    CTREHelper.checkStatus(
        motorSim.setRotorVelocity(motor_velocity), "azimuthSim::setRotorVelocity");
    CTREHelper.checkStatus(
        motorSim.setRawRotorPosition(motor_position), "azimuthSim::setRotorVelocity");

    CTREHelper.checkStatus(
        encoderSim.setRawPosition(
            m_azimuthSim.getAngleRadians() / UnitConversion.RADIANS_PER_ROTATION),
        "encoderSim::setRawPosition");
    CTREHelper.checkStatus(
        encoderSim.setVelocity(
            m_azimuthSim.getAngularVelocityRadiansPerSecond()
                / UnitConversion.RADIANS_PER_ROTATION),
        "encoderSim::setVelocity");
  }
}
