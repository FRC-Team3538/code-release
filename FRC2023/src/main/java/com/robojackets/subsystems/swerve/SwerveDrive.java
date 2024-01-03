package com.robojackets.subsystems.swerve;

import com.robojackets.lib.UnitConversion;
import com.robojackets.lib.controller.CustomHolonomicController;
import com.robojackets.lib.limelight.LimelightHelpers;
import com.robojackets.lib.limelight.LimelightHelpers.LimelightResults;
import com.robojackets.lib.limelight.LimelightHelpers.Results;
import com.robojackets.lib.trajectory.TrajectoryState;
import com.robojackets.lib.wpimath.estimator.SwerveDrivePoseEstimator;
import com.robojackets.subsystems.gyro.GyroIO;
import com.robojackets.subsystems.gyro.GyroIOInputsAutoLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final ModuleContainer modules;
  private final GyroIO gyroIO;
  public final GyroIOInputsAutoLogged ioInputs = new GyroIOInputsAutoLogged();

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator estimator;

  private final CustomHolonomicController trajectoryController;

  private final Field2d field = new Field2d();

  private Pose2d commandPose = new Pose2d();

  private final double kMaxSpeedLinearMetersPerSecond = 15 * UnitConversion.METERS_PER_FOOT;
  private final double kMaxSpeedAngularRadiansPerSecond = 2 * UnitConversion.RADIANS_PER_ROTATION;
  private final double kMaxAccelerationLinearMetersPerSecondSquared =
      20 * UnitConversion.METERS_PER_FOOT;

  private LimelightResults lastResults;

  public SwerveDrive(ModuleContainer modules, GyroIO gyroIO) {
    this.modules = modules;
    this.gyroIO = gyroIO;

    this.ioInputs.setQuaternion(new double[] {0, 0, 0, 0});

    this.kinematics = new SwerveDriveKinematics(modules.getLocations());

    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRawHeading3d(),
            modules.getOdometryData(),
            new Pose3d(),
            VecBuilder.fill(0.05, 0.05, 0.05, 0.01),
            VecBuilder.fill(0.1, 0.1, 0.1, 0.1));

    this.trajectoryController =
        new CustomHolonomicController(
            new PIDController(5, 0, 0), new PIDController(5, 0, 0), new PIDController(2, 0, 0));

    this.lastResults = new LimelightResults();
    this.lastResults.targetingResults = new Results();

    SmartDashboard.putData(field);
  }

  private Rotation2d getRawHeading() {
    return Rotation2d.fromRadians(ioInputs.getYawRadians());
  }

  private Rotation3d getRawHeading3d() {
    if (RobotBase.isReal()) {
      var quat = ioInputs.getQuaternion();
      return new Rotation3d(new Quaternion(quat[0], quat[1], quat[2], quat[3]));
    } else {
      return new Rotation3d(0, 0, ioInputs.getYawRadians());
    }
  }

  public void resetYaw(Rotation3d heading) {
    var pose = estimator.getEstimatedPosition3d();

    estimator.resetPosition(
        getRawHeading3d(), modules.getOdometryData(), new Pose3d(pose.getTranslation(), heading));
  }

  public void resetPose(Pose2d pose) {
    commandPose = pose;

    estimator.resetPosition(getRawHeading3d(), modules.getOdometryData(), new Pose3d(pose));
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition3d().toPose2d();
  }

  public void addVisionMeasurement(double timestamp, Pose3d measurement, Vector<N4> stddevs) {
    estimator.addVisionMeasurement(measurement, timestamp, stddevs);

    Logger.getInstance().recordOutput("Swerve/Estimator/Pose", estimator.getEstimatedPosition3d());
  }

  public void addVisionMeasurement(double timestamp, Pose3d measurement) {
    estimator.addVisionMeasurement(measurement, timestamp);

    Logger.getInstance().recordOutput("Swerve/Estimator/Pose", estimator.getEstimatedPosition3d());
  }

  public void maybeApplyVisionUpdate() {
    var currentResults = maybeGetNewerVisionEstimate(lastResults, "limelight");

    if (currentResults.isPresent()) {
      var total_latency =
          currentResults.get().targetingResults.latency_capture
              + currentResults.get().targetingResults.latency_pipeline
              + currentResults.get().targetingResults.latency_jsonParse;
      var estimate_timestamp =
          currentResults.get().targetingResults.timestamp_RIOFPGA_capture - total_latency * 0.001;

      lastResults = currentResults.get();

      if (!estimator.isRecent(estimate_timestamp)) {
        return;
      }

      Logger.getInstance().recordOutput("vision/latency", total_latency);
      Logger.getInstance()
          .recordOutput("vision/capture", lastResults.targetingResults.timestamp_RIOFPGA_capture);
      Logger.getInstance()
          .recordOutput("vision/latency/capture", lastResults.targetingResults.latency_capture);
      Logger.getInstance()
          .recordOutput("vision/latency/pipeline", lastResults.targetingResults.latency_pipeline);
      Logger.getInstance()
          .recordOutput("vision/latency/parse", lastResults.targetingResults.latency_jsonParse);
      Logger.getInstance()
          .recordOutput("vision/estimate", lastResults.targetingResults.getBotPose3d_wpiBlue());
      Logger.getInstance().recordOutput("vision/reference", estimator.sampleAt(estimate_timestamp));
      Logger.getInstance().recordOutput("vision/timestamp", estimate_timestamp);
      Logger.getInstance()
          .recordOutput(
              "vision/fiducial_count", lastResults.targetingResults.targets_Fiducials.length);

      if (new Transform3d(
                      estimator.sampleAt(estimate_timestamp),
                      currentResults.get().targetingResults.getBotPose3d_wpiBlue())
                  .getTranslation()
                  .getNorm()
              > 1
          && !DriverStation.isDisabled()) {
        return;
      }

      if (lastResults.targetingResults.targets_Fiducials.length >= 2) {

        this.estimator.addVisionMeasurement(
            lastResults.targetingResults.getBotPose3d_wpiBlue(),
            estimate_timestamp,
            VecBuilder.fill(0.1, 0.1, 0.1, 0.1));
      } else {
        this.estimator.addVisionMeasurement(
            lastResults.targetingResults.getBotPose3d_wpiBlue(),
            estimate_timestamp,
            VecBuilder.fill(1, 1, 1, 1));
      }
    }
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(ioInputs);
    Logger.getInstance().processInputs("Swerve/Gyro", ioInputs);

    modules.forEach(SwerveModule::periodic);

    maybeApplyVisionUpdate();

    var newPose = estimator.update(getRawHeading3d(), modules.getOdometryData());
    var speeds = kinematics.toChassisSpeeds(modules.getStates());

    Logger.getInstance().recordOutput("Swerve/States", modules.getStates());
    Logger.getInstance()
        .recordOutput(
            "Swerve/Speeds",
            new double[] {
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
            });
    Logger.getInstance().recordOutput("Swerve/Estimator/Pose", newPose);
    Logger.getInstance().recordOutput("Swerve/Estimator/Pose2d", newPose.toPose2d());
    Logger.getInstance().recordOutput("Swerve/Command/Pose", commandPose);

    field.getRobotObject().setPose(newPose.toPose2d());
  }

  public void DriveTrajectoryState(TrajectoryState state) {
    Logger.getInstance().recordOutput("Swerve/TrajectoryState/time", state.getTimestamp());
    Logger.getInstance()
        .recordOutput(
            "Swerve/TrajectoryState/speeds",
            new double[] {
              state.getChassisSpeeds().vxMetersPerSecond,
              state.getChassisSpeeds().vyMetersPerSecond,
              state.getChassisSpeeds().omegaRadiansPerSecond
            });
    Logger.getInstance().recordOutput("Swerve/TrajectoryState/pose", state.getPose());
    field.getObject("trajectory").setPose(state.getPose());

    DriveRobotCentric(
        trajectoryController.calculate(
            estimator.getEstimatedPosition3d().toPose2d(),
            state.getPose(),
            state.getChassisSpeeds()));
  }

  public void DriveFieldCentric(ChassisSpeeds speeds) {
    Logger.getInstance()
        .recordOutput(
            "Swerve/Command/Speeds/FieldCentric",
            new double[] {
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
            });
    DriveRobotCentric(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            getPose()
                .getRotation())); // DriveRobotCentric in here means modules.Drive(states) happens
  }

  public void DriveRobotCentric(ChassisSpeeds speeds) {

    Logger.getInstance()
        .recordOutput(
            "Swerve/Command/Speeds/RobotCentric",
            new double[] {
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
            });

    // var error = getPose().log(commandPose);

    // var error_velocity = new ChassisSpeeds(error.dx / 0.02, error.dy / 0.02, error.dtheta /
    // 0.02);

    // speeds.vxMetersPerSecond += error_velocity.vxMetersPerSecond;
    // speeds.vyMetersPerSecond += error_velocity.vyMetersPerSecond;
    // speeds.omegaRadiansPerSecond += error_velocity.omegaRadiansPerSecond;

    var states = kinematics.toSwerveModuleStates(speeds);

    Logger.getInstance().recordOutput("Swerve/Command/OriginalStates", states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        speeds,
        kMaxSpeedLinearMetersPerSecond,
        kMaxSpeedLinearMetersPerSecond * 14.0 / 15.0,
        kMaxSpeedAngularRadiansPerSecond);

    Logger.getInstance().recordOutput("Swerve/Command/DesaturatedStates", states);

    modules.Drive(states); // using the Drivecommand from ModuleContainer....
  }

  private static SwerveModuleState[] cross =
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)), // FL
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45.0)), // FR
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45.0)), // BL
        new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)), // BR
      };

  public void Cross() {
    modules.Drive(cross);
  }

  public void DriveHoldingHeading(Rotation2d heading, ChassisSpeeds speeds) {
    var referencePose =
        new Pose2d(estimator.getEstimatedPosition3d().toPose2d().getTranslation(), heading);
    speeds.omegaRadiansPerSecond = 0;

    DriveRobotCentric(
        trajectoryController.calculate(
            estimator.getEstimatedPosition3d().toPose2d(), referencePose, speeds));
  }

  @Override
  public void simulationPeriodic() {
    var speeds = kinematics.toChassisSpeeds(modules.getStates());
    this.gyroIO.SimPeriodic(new Rotation3d(0, 0, speeds.omegaRadiansPerSecond * 0.02));

    modules.forEach(SwerveModule::simPeriodic);
  }

  private Optional<LimelightResults> maybeGetNewerVisionEstimate(
      LimelightResults results, String limelight) {
    if (!LimelightHelpers.tableExists(limelight)) {
      return Optional.empty();
    }

    var currentResults = LimelightHelpers.getLatestResults(limelight);

    if (currentResults.targetingResults == null) {
      return Optional.empty();
    }

    if (!currentResults.targetingResults.valid) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.timestamp_LIMELIGHT_publish
        <= results.targetingResults.timestamp_LIMELIGHT_publish) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.botpose_wpiblue[0] == 0
        && currentResults.targetingResults.botpose_wpiblue[1] == 0
        && currentResults.targetingResults.botpose_wpiblue[2] == 0
        && currentResults.targetingResults.botpose_wpiblue[3] == 0
        && currentResults.targetingResults.botpose_wpiblue[4] == 0
        && currentResults.targetingResults.botpose_wpiblue[5] == 0) {
      return Optional.empty();
    }

    // max 4 targets, typically 1-2
    for (var fiducial : currentResults.targetingResults.targets_Fiducials) {
      if (fiducial.getRobotPose_TargetSpace().getTranslation().getNorm() > 7) {
        return Optional.empty();
      }
    }

    // return Optional.empty();

    return Optional.of(currentResults);
  }
}
