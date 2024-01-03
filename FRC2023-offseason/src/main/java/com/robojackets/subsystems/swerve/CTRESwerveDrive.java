package com.robojackets.subsystems.swerve;

import static com.robojackets.ctre.SwerveRequest.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.robojackets.ctre.SwerveDrivetrain;
import com.robojackets.ctre.SwerveDrivetrainConstants;
import com.robojackets.ctre.SwerveModule.DriveRequestType;
import com.robojackets.ctre.SwerveModule.SteerRequestType;
import com.robojackets.ctre.SwerveModuleConstants;
import com.robojackets.lib.limelight.LimelightHelpers;
import com.robojackets.lib.limelight.LimelightHelpers.LimelightResults;
import com.robojackets.lib.limelight.LimelightHelpers.Results;
import com.robojackets.lib.trajectory.Trajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class CTRESwerveDrive extends SwerveDrivetrain {
  private final Field2d field = new Field2d();

  private FieldCentric fieldCentric =
      new FieldCentric()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic);
  private RobotCentric robotCentric =
      new RobotCentric()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic);
  private SwerveDriveBrake brake =
      new SwerveDriveBrake()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic);
  private FieldCentricFacingAngle ffa =
      new FieldCentricFacingAngle()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic);
  private RobotCentricFacingAngle rfa =
      new RobotCentricFacingAngle()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic);
  private FollowTrajectory followTrajectory =
      new FollowTrajectory()
          .withDriveRequestType(
              Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagic)
          .withFieldObject(field.getObject("trajectory"));

  private LimelightResults lastResults;

  public CTRESwerveDrive(
      SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... modules) {
    super(driveConstants, 250, modules);

    var mountOrientation = new MountPoseConfigs();
    mountOrientation.MountPosePitch = 2.4379966259002686;
    mountOrientation.MountPoseRoll = -0.5633166432380676;
    mountOrientation.MountPoseYaw = 1.535753607749939;

    m_pigeon2.getConfigurator().apply(mountOrientation);

    rfa.HeadingController.setP(10);
    ffa.HeadingController.setP(10);

    rfa.HeadingController.setD(1);
    ffa.HeadingController.setD(1);

    followTrajectory.XController.setP(5);
    followTrajectory.YController.setP(5);
    followTrajectory.HeadingController.setP(4);

    this.lastResults = new LimelightResults();
    this.lastResults.targetingResults = new Results();

    registerTelemetry(this::recordTelemetry);

    SmartDashboard.putData(field);
  }

  public void resetYaw(Rotation2d heading) {
    var pose = getState().Pose;
    resetPose(
        new Pose3d(
            pose.getTranslation(),
            new Rotation3d(
                pose.getRotation().getX(), pose.getRotation().getY(), heading.getRadians())));
  }

  public void resetYaw(Rotation3d heading) {
    seedFieldRelative(new Pose3d(getState().Pose.getTranslation(), heading));
  }

  public void resetPose(Pose2d pose) {
    seedFieldRelative(new Pose3d(pose));
  }

  public void resetPose(Pose3d pose) {
    seedFieldRelative(pose);
  }

  public Pose2d getPose() {
    return getState().Pose.toPose2d();
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

      if (!isRecent(estimate_timestamp)) {
        return;
      }

      Logger.recordOutput("vision/latency", total_latency);
      Logger.recordOutput("vision/capture", lastResults.targetingResults.timestamp_RIOFPGA_capture);
      Logger.recordOutput("vision/latency/capture", lastResults.targetingResults.latency_capture);
      Logger.recordOutput("vision/latency/pipeline", lastResults.targetingResults.latency_pipeline);
      Logger.recordOutput("vision/latency/parse", lastResults.targetingResults.latency_jsonParse);
      Logger.recordOutput("vision/estimate", lastResults.targetingResults.getBotPose3d_wpiBlue());
      Logger.recordOutput("vision/reference", samplePoseAt(estimate_timestamp));
      Logger.recordOutput("vision/timestamp", estimate_timestamp);
      Logger.recordOutput(
          "vision/fiducial_count", lastResults.targetingResults.targets_Fiducials.length);

      var measurement = currentResults.get().targetingResults.getBotPose3d_wpiBlue();
      var sampleRotation = samplePoseAt(estimate_timestamp).getRotation();

      // trust the pigeon
      if (measurement.getRotation().minus(sampleRotation).getAngle() > 0.1
          && !DriverStation.isDisabled()) {
        return;
      }

      if (lastResults.targetingResults.targets_Fiducials.length >= 2) {

        addVisionMeasurement(measurement, estimate_timestamp, VecBuilder.fill(0.1, 0.1, 0.1, 0.1));
      } else {
        addVisionMeasurement(measurement, estimate_timestamp, VecBuilder.fill(1, 1, 1, 1));
      }
    }
  }

  private double lastTime = 0;

  public void recordTelemetry(SwerveDriveState state) {
    try {
      Logger.recordOutput("Swerve/OdometryThread/Period", state.OdometryPeriod);
      Logger.recordOutput("Swerve/Estimator/Heading", state.Pose.getRotation().toRotation2d());
      Logger.recordOutput("Swerve/Estimator/Pose", state.Pose);
      Logger.recordOutput("Swerve/States", state.ModuleStates);
      Logger.recordOutput("Swerve/OdometryThread/SuccessfulDaqs", state.SuccessfulDaqs);
      Logger.recordOutput("Swerve/OdometryThread/FailedDaqs", state.FailedDaqs);
      Logger.recordOutput(
          "Swerve/Speeds",
          new double[] {
            state.ChassisSpeeds.vxMetersPerSecond,
            state.ChassisSpeeds.vyMetersPerSecond,
            state.ChassisSpeeds.omegaRadiansPerSecond
          });
    } catch (Exception ex) {
      DriverStation.reportError(ex.getLocalizedMessage(), ex.getStackTrace());
    }

    field.getRobotObject().setPose(state.Pose.toPose2d());

    if (Utils.isSimulation()) {
      var currentTime = Utils.getCurrentTimeSeconds();
      updateSimState(currentTime - lastTime, 12);
      lastTime = currentTime;
    }
  }

  public void periodic() {
    maybeApplyVisionUpdate();
  }

  public void DriveTrajectory(Trajectory traj) {
    setControl(followTrajectory.withTrajectory(traj).withStartTime(Utils.getCurrentTimeSeconds()));
  }

  public void DriveFieldCentric(ChassisSpeeds speeds) {
    setControl(
        fieldCentric
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public void DriveRobotCentric(ChassisSpeeds speeds) {
    setControl(
        robotCentric
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public void Cross() {
    setControl(brake);
  }

  public void DriveRobotCentricHoldingHeading(Rotation2d heading, ChassisSpeeds speeds) {
    setControl(
        rfa.withTargetDirection(heading)
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond));
  }

  public void DriveFieldCentricHoldingHeading(Rotation2d heading, ChassisSpeeds speeds) {
    setControl(
        ffa.withTargetDirection(heading)
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond));
  }

  public void simulationPeriodic() {
    // updateSimState(0.02, 12);
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
