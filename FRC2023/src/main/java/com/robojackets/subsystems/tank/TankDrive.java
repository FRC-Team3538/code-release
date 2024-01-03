package com.robojackets.subsystems.tank;

import com.robojackets.Constants;
import com.robojackets.lib.ArcadeDriveInput;
import com.robojackets.lib.controller.DifferentialDriveFeedforward;
import com.robojackets.subsystems.gearbox.GearboxIO;
import com.robojackets.subsystems.gearbox.GearboxIOInputsAutoLogged;
import com.robojackets.subsystems.gyro.GyroIO;
import com.robojackets.subsystems.gyro.GyroIOInputsAutoLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TankDrive extends SubsystemBase {
  private final TankDriveIO io;
  private final DriveIOInputsAutoLogged ioInputs = new DriveIOInputsAutoLogged();
  private final GearboxIOInputsAutoLogged leftIOInputs = new GearboxIOInputsAutoLogged();
  private final GearboxIOInputsAutoLogged rightIOInputs = new GearboxIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveFeedforward feedforward;
  private final LTVDifferentialDriveController controller;
  private final DifferentialDrivePoseEstimator observer;

  public TankDrive(
      DifferentialDriveKinematics kinematics,
      DifferentialDriveFeedforward feedforward,
      LTVDifferentialDriveController controller,
      DifferentialDrivePoseEstimator observer,
      TankDriveIO io) {
    this.kinematics = kinematics;
    this.feedforward = feedforward;
    this.controller = controller;
    this.observer = observer;
    this.io = io;
  }

  public TankDrive() {
    double trackWidthMeters;
    double linearKv;
    double linearKa;
    double angularKv;
    double angularKa;
    LinearSystem<N2, N2, N2> plant;

    switch (Constants.getRobot()) {
      case ROBOT_ROVER:
        trackWidthMeters = 0.78107;
        linearKv = 2.643;
        linearKa = 0.16209;
        angularKv = 2.7351;
        angularKa = 0.035228;
        plant =
            LinearSystemId.identifyDrivetrainSystem(
                linearKv, linearKa, angularKv, angularKa, trackWidthMeters);

        io = new FalconTankDriveIO();
        break;
      case ROBOT_SIMBOT:
      default:
        trackWidthMeters = 0.78107;
        linearKv = 2.643;
        linearKa = 0.16209;
        angularKv = 2.7351;
        angularKa = 0.035228;
        plant =
            LinearSystemId.identifyDrivetrainSystem(
                linearKv, linearKa, angularKv, angularKa, trackWidthMeters);

        io = new TankDriveIO(new GearboxIO() {}, new GearboxIO() {}, new GyroIO()) {};

        break;
    }

    kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    feedforward = new DifferentialDriveFeedforward(plant);
    controller =
        new LTVDifferentialDriveController(
            plant,
            trackWidthMeters,
            VecBuilder.fill(0.0625, 0.125, 2.5, 0.95, 0.95),
            VecBuilder.fill(12, 12),
            trackWidthMeters);
    observer =
        new DifferentialDrivePoseEstimator(
            kinematics,
            Rotation2d.fromRadians(gyroIOInputs.getYawRadians()),
            ioInputs.getLeftPositionMeters(),
            ioInputs.getRightPositionMeters(),
            new Pose2d());
  }

  public void Arcade(ArcadeDriveInput voltage) {
    io.setVoltage(
        new DifferentialDriveWheelVoltages(
            voltage.forward() * 12 + voltage.turn() * 12,
            voltage.forward() * 12 - voltage.turn() * 12));
  }

  public void Tank(DifferentialDriveWheelSpeeds speeds) {
    Tank(
        speeds,
        feedforward.calculate(
            ioInputs.getLeftVelocityMetersPerSecond(),
            speeds.leftMetersPerSecond,
            ioInputs.getRightVelocityMetersPerSecond(),
            speeds.rightMetersPerSecond));
  }

  public void Tank(
      DifferentialDriveWheelSpeeds wheelSpeeds, DifferentialDriveWheelVoltages feedforward) {
    io.setWheelSpeeds(wheelSpeeds, feedforward);
  }

  public void Drive(ChassisSpeeds speeds) {
    Tank(kinematics.toWheelSpeeds(speeds));
  }

  public void Drive(Trajectory.State goalState, Pose2d currentPose) {
    var voltages =
        controller.calculate(
            currentPose,
            ioInputs.getLeftVelocityMetersPerSecond(),
            ioInputs.getRightVelocityMetersPerSecond(),
            goalState);
    var speeds =
        kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                goalState.velocityMetersPerSecond,
                0,
                goalState.velocityMetersPerSecond * goalState.curvatureRadPerMeter));

    Tank(speeds, voltages);
  }

  public void setBrakeMode(boolean BrakeMode) {
    io.setBrakeMode(BrakeMode);
  }

  public void resetPose(Pose2d pose) {
    observer.resetPosition(
        Rotation2d.fromRadians(gyroIOInputs.getYawRadians()),
        ioInputs.getLeftPositionMeters(),
        ioInputs.getRightPositionMeters(),
        pose);
  }

  @Override
  public void periodic() {
    io.updateInputs(ioInputs, leftIOInputs, rightIOInputs, gyroIOInputs);
    Logger.getInstance().processInputs("DriveIO", ioInputs);
    Logger.getInstance().processInputs("DriveIO/Left", leftIOInputs);
    Logger.getInstance().processInputs("DriveIO/Right", rightIOInputs);

    Logger.getInstance()
        .recordOutput(
            "DriveIO/Pose",
            observer.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(gyroIOInputs.getYawRadians()),
                ioInputs.getLeftPositionMeters(),
                ioInputs.getRightPositionMeters()));
  }
}
