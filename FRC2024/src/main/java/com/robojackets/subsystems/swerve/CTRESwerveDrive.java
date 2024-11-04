package com.robojackets.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentric;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentricFacingAngle;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentricFacingPoint;
import com.robojackets.ctre.RJSwerveRequest.RJFollowTrajectory;
import com.robojackets.ctre.RJSwerveRequest.RJHoldPose;
import com.robojackets.ctre.RJSwerveRequest.RJRobotCentric;
import com.robojackets.ctre.RJSwerveRequest.RJRobotCentricFacingAngle;
import com.robojackets.ctre.RJSwerveRequest.SwerveSteerImpulseResponse;
import com.robojackets.ctre.RJSwerveRequest.SysIdSwerveRotationAmps;
import com.robojackets.ctre.RJSwerveRequest.SysIdSwerveSteerGainsAmps;
import com.robojackets.ctre.RJSwerveRequest.SysIdSwerveTranslationAmps;
import com.robojackets.ctre.SwerveDrivetrain;
import com.robojackets.ctre.SwerveDrivetrainConstants;
import com.robojackets.ctre.SwerveModule.DriveRequestType;
import com.robojackets.ctre.SwerveModule.SteerRequestType;
import com.robojackets.ctre.SwerveModuleConstants;
import com.robojackets.ctre.SwerveRequest;
import com.robojackets.ctre.SwerveRequest.FieldCentricFacingAngle;
import com.robojackets.ctre.SwerveRequest.SwerveControlRequestParameters;
import com.robojackets.ctre.SwerveRequest.SwerveDriveBrake;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveRotation;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveSteerGains;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveTranslation;
import com.robojackets.lib.trajectory.MotionProfile;
import com.robojackets.lib.trajectory.RotationMotionProfileFollower;
import com.robojackets.lib.trajectory.TrapezoidCurve;
import com.robojackets.oi.ControlScheme;
import com.robojackets.subsystems.vision.VisionModule;
import com.robojackets.subsystems.vision.VisionModule.VisionEstimate;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CTRESwerveDrive extends SwerveDrivetrain implements Subsystem {
  private final Field2d field = new Field2d();

  private final VisionModule camera;

  private final TrapezoidCurve.Constraints rotationConstraints =
      new TrapezoidCurve.Constraints(2 * Math.PI)
          .withMaxVelocity(Math.PI); // accelerate at 2 pi rad/s/s, set max velocity to pi rad/s

  private final RotationMotionProfileFollower<MotionProfile<TrapezoidCurve>>
      stopTurningProfileFollower = rotationConstraints.asVelocityMotionProfile().rotationFollower();
  private final RotationMotionProfileFollower<MotionProfile<TrapezoidCurve>>
      facingAngleProfileFollower =
          rotationConstraints.asFullStateMotionProfile().rotationFollower();

  private final DriveRequestType driveRequestType =
      Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity;
  private final SteerRequestType steerRequestType = SteerRequestType.MotionMagic;

  private IOInputs inputs = new IOInputs();

  private RJFieldCentric fieldCentric =
      new RJFieldCentric()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private RJRobotCentric robotCentric =
      new RJRobotCentric()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private SwerveDriveBrake brake =
      new SwerveDriveBrake()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private RJFieldCentricFacingAngle fieldCentricStopRotation =
      new RJFieldCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(stopTurningProfileFollower)
          .withRotationalDeadband(0.1);
  private FieldCentricFacingAngle fieldCentricFacingAngle =
      new FieldCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private RJFieldCentricFacingAngle fieldCentricFacingPose =
      new RJFieldCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(facingAngleProfileFollower);
  private RJRobotCentricFacingAngle robotCentricStopRotation =
      new RJRobotCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(stopTurningProfileFollower)
          .withRotationalDeadband(0.1);
  private RJRobotCentricFacingAngle robotCentricFacingAngle =
      new RJRobotCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(facingAngleProfileFollower);
  private RJRobotCentricFacingAngle robotCentricFacingPose =
      new RJRobotCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(facingAngleProfileFollower);
  private RJFollowTrajectory followTrajectory =
      new RJFollowTrajectory()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withTelemetryFunc(state -> inputs.TrajectoryState = state);
  private RJHoldPose holdPose =
      new RJHoldPose()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private RJFieldCentricFacingPoint fieldCentricFacingPoint =
      new RJFieldCentricFacingPoint()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);

  private SwerveSteerImpulseResponse steerImpulseResponse = new SwerveSteerImpulseResponse();

  private SysIdSwerveTranslationAmps sysidTranslationRequest = new SysIdSwerveTranslationAmps();

  private SysIdSwerveRotationAmps sysidRotationRequest = new SysIdSwerveRotationAmps();

  private SysIdSwerveSteerGainsAmps sysidSteerGainsRequest = new SysIdSwerveSteerGainsAmps();

  private SysIdSwerveSteerGains sysidSteerGainsStopRequest = new SysIdSwerveSteerGains();
  private SysIdSwerveTranslation sysidSwerveTranslationStopRequest = new SysIdSwerveTranslation();
  private SysIdSwerveRotation sysidSwerveRotationStopRequest = new SysIdSwerveRotation();

  {
    sysidSteerGainsRequest.m_ampRequest.withUpdateFreqHz(0);
    sysidSteerGainsRequest.m_voltRequest.withOverrideBrakeDurNeutral(true).withUpdateFreqHz(0);
    sysidTranslationRequest.m_ampRequest.withUpdateFreqHz(0);
    sysidRotationRequest.m_ampRequest.withUpdateFreqHz(0);

    sysidSteerGainsStopRequest.m_voltRequest.withOverrideBrakeDurNeutral(true).withUpdateFreqHz(0);
    sysidSwerveTranslationStopRequest
        .m_voltRequest
        .withOverrideBrakeDurNeutral(true)
        .withUpdateFreqHz(0);
    sysidSwerveRotationStopRequest
        .m_voltRequest
        .withOverrideBrakeDurNeutral(true)
        .withUpdateFreqHz(0);
  }

  private SysIdRoutine m_driveTranslationRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(5),
              Seconds.of(3),
              state -> SignalLogger.writeString("drive-translation-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidSwerveTranslationStopRequest.withVolts(volts)),
              null,
              this));

  private SysIdRoutine m_driveTranslationAmpsRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(10).per(Second),
              Volts.of(15),
              Seconds.of(3),
              state -> SignalLogger.writeString("drive-translation-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidTranslationRequest.withAmps(Amps.of(volts.in(Volts)))),
              null,
              this));

  private SysIdRoutine m_driveRotationRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(5),
              Seconds.of(3),
              state -> SignalLogger.writeString("drive-rotation-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidSwerveRotationStopRequest.withVolts(volts)),
              null,
              this));

  private SysIdRoutine m_steerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(5),
              Seconds.of(5),
              state -> SignalLogger.writeString("steer-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidSteerGainsStopRequest.withVolts(volts)), null, this));

  public CTRESwerveDrive(
      VisionModule camera,
      SwerveDrivetrainConstants driveConstants,
      SwerveModuleConstants... modules) {
    super(driveConstants, 250, modules);

    this.camera = camera;

    var mountOrientation = new MountPoseConfigs();
    mountOrientation.MountPosePitch = -0.1023130938410759;
    mountOrientation.MountPoseRoll = -0.3815140128135681;
    mountOrientation.MountPoseYaw = 96.27735137939453;

    m_pigeon2.getConfigurator().apply(mountOrientation);

    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    robotCentricFacingAngle.HeadingController.setP(4);
    fieldCentricFacingAngle.HeadingController.setP(4);

    robotCentricFacingAngle.HeadingController.setD(0.1);
    fieldCentricFacingAngle.HeadingController.setD(0.1);

    robotCentricStopRotation.HeadingController.setP(4);
    fieldCentricStopRotation.HeadingController.setP(4);

    robotCentricStopRotation.HeadingController.setD(0.1);
    fieldCentricStopRotation.HeadingController.setD(0.1);

    fieldCentricFacingPoint.HeadingController.setP(6);
    fieldCentricFacingPoint.HeadingController.setD(0.1);

    followTrajectory.XController.setP(5);
    followTrajectory.YController.setP(5);
    followTrajectory.HeadingController.setP(4);

    holdPose.XController.setP(5);
    holdPose.YController.setP(5);
    holdPose.HeadingController.setP(4);

    registerTelemetry(this::recordTelemetry);

    SmartDashboard.putData(field);
  }

  public void resetYaw() {
    setOperatorForwardToCurrentDirection();
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

  private double lastTime = 0;

  public void recordTelemetry(SwerveDriveState state) {
    field.getRobotObject().setPose(state.Pose.toPose2d());

    if (Utils.isSimulation()) {
      var currentTime = Utils.getCurrentTimeSeconds();
      updateSimState(currentTime - lastTime, 12);
      lastTime = currentTime;
    }
  }

  private boolean useVision = true;

  public boolean VisionStatus() {
    return useVision;
  }

  public void DisableVision() {
    useVision = false;
  }

  public void EnableVision() {
    useVision = true;
  }

  public void periodic() {

    inputs.SwerveDriveState = getState();
    inputs.ControlParameters = getControlParameters();
    var visionEstimate = camera.getNewPoseEstimate(inputs.SwerveDriveState.Pose);
    inputs.VisionEstimate = visionEstimate.orElse(inputs.VisionEstimate);

    Logger.processInputs("SwerveDrive", inputs);

    if (useVision) {
      processVisionUpdate(visionEstimate);
    }

    if (!stopTurningRequestRunning.getAsBoolean()) {
      seedStopTurningState();
    }

    if (!facingAngleRequestRunning.getAsBoolean()) {
      seedFacingAngleState();
    }
  }

  private void processVisionUpdate(Optional<VisionEstimate> visionEstimate) {
    if (visionEstimate.isEmpty()) {
      Logger.recordOutput("vision/applyEstimate", false);
      return;
    }

    if (!isWithinBuffer(inputs.VisionEstimate.getTimestamp())) {
      Logger.recordOutput("vision/applyEstimate", false);
      return;
    }

    if (DriverStation.isEnabled()) {
      if (Math.abs(inputs.VisionEstimate.getEstimate().getZ()) > 0.1) {
        // not on the ground; that ain't us
        Logger.recordOutput("vision/applyEstimate", false);
        return;
      }

      var stdevs = inputs.VisionEstimate.getStdDevs();
      if (Math.sqrt(stdevs.get(0) * stdevs.get(0) + stdevs.get(1) * stdevs.get(1)) > 2) {
        Logger.recordOutput("vision/applyEstimate", false);
        return;
      }

      Logger.recordOutput("vision/applyEstimate", true);

      addVisionMeasurement(
          inputs.VisionEstimate.getEstimate(),
          inputs.VisionEstimate.getTimestamp(),
          inputs.VisionEstimate.getStdDevs());
    } else {
      Logger.recordOutput("vision/applyEstimate", true);

      addVisionMeasurement(
          inputs.VisionEstimate.getEstimate(),
          inputs.VisionEstimate.getTimestamp(),
          inputs.VisionEstimate.getStdDevs());
    }
  }

  private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command FollowPath(ChoreoTrajectory traj) {
    return Commands.runOnce(() -> followTrajectory.stopwatch.restart())
        .andThen(
            waitSeconds(traj.getTotalTime())
                .deadlineWith(applyRequest(() -> followTrajectory.withTrajectory(traj))))
        .withName("Swerve::FollowPath");
  }

  public Command ImpulseResponse() {
    return applyRequest(() -> steerImpulseResponse).withName("Swerve::SteerImpulseResponse");
  }

  public Command HoldPose(Pose2d pose) {
    return applyRequest(() -> holdPose.withTargetPose(pose)).withName("Swerve::HoldPose");
  }

  public Command FieldCentricFacingPoint(ControlScheme io, Supplier<Translation2d> point) {
    return applyRequest(
            () -> {
              if (VisionStatus()) {
                return fieldCentricFacingPoint
                    .withSpeeds(io.getSwerveDriveInput())
                    .withTargetPoint(point.get());
              } else {
                return fieldCentric.withSpeeds(io.getSwerveDriveInput());
              }
            })
        .withName("Swerve::FacePoint");
  }

  public Command FieldCentricFacingAngle(ControlScheme io, Rotation2d heading) {
    return applyRequest(
            () -> {
              var speeds = io.getSwerveDriveInput();
              if (VisionStatus()) {
                return fieldCentricFacingAngle
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withTargetDirection(heading);
              } else {
                return fieldCentric.withSpeeds(speeds);
              }
            })
        .withName("Swerve::FaceHeading");
  }

  public Command LockWheels() {
    return applyRequest(() -> brake).withName("Swerve::LockWheels");
  }

  public Command TeleopDrive(ControlScheme io) {
    return applyRequest(
            () -> {
              ChassisSpeeds motion = io.getSwerveDriveInput();

              if (io.useRobotCentric()) {
                // if (motion.omegaRadiansPerSecond == 0) {
                // return robotCentricStopRotation.withSpeeds(motion);
                // } else {
                return robotCentric.withSpeeds(motion);
                // }
              } else {
                // if (motion.omegaRadiansPerSecond == 0) {
                // return fieldCentricStopRotation.withSpeeds(motion);
                // } else {
                return fieldCentric.withSpeeds(motion);
                // }
              }
            })
        .withName("Swerve::TeleopDrive");
  }

  public void RobotCentricDrive(ChassisSpeeds motion) {
    this.setControl(robotCentric.withSpeeds(motion));
  }

  public Command sysidSteer() {
    return m_steerRoutine
        .quasistatic(Direction.kForward)
        .andThen(
            applyRequest(() -> sysidSteerGainsStopRequest.withVolts(Volts.of(0))).withTimeout(1))
        .andThen(m_steerRoutine.quasistatic(Direction.kReverse))
        .andThen(
            applyRequest(() -> sysidSteerGainsStopRequest.withVolts(Volts.of(0))).withTimeout(1))
        .andThen(m_steerRoutine.dynamic(Direction.kForward).withTimeout(1))
        .andThen(
            applyRequest(() -> sysidSteerGainsStopRequest.withVolts(Volts.of(0))).withTimeout(1))
        .andThen(m_steerRoutine.dynamic(Direction.kReverse).withTimeout(1))
        .andThen(
            applyRequest(() -> sysidSteerGainsStopRequest.withVolts(Volts.of(0))).withTimeout(1))
        .withName("Swerve::SysIdSteer");
  }

  public Command sysidDriveTranslation() {
    return m_driveTranslationRoutine
        .quasistatic(Direction.kForward)
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveTranslationRoutine.quasistatic(Direction.kReverse))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveTranslationRoutine.dynamic(Direction.kForward).withTimeout(2))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveTranslationRoutine.dynamic(Direction.kReverse).withTimeout(2))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .withName("Swerve::SysIdDriveTranslation");
  }

  public Command sysidDriveTranslationAmps() {
    return
    // runOnce(() -> origin = getState().Pose.toPose2d())
    // .andThen(
    m_driveTranslationAmpsRoutine
        .quasistatic(Direction.kForward)
        .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 1)
        // )
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 1))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .dynamic(Direction.kForward)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 2))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .dynamic(Direction.kReverse)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 2))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .withName("Swerve::SysIdDriveTranslationAmps");
  }

  public Command sysidDriveRotation() {
    return m_driveRotationRoutine
        .quasistatic(Direction.kForward)
        .andThen(
            applyRequest(() -> sysidSwerveRotationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveRotationRoutine.quasistatic(Direction.kReverse))
        .andThen(
            applyRequest(() -> sysidSwerveRotationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveRotationRoutine.dynamic(Direction.kForward).withTimeout(2))
        .andThen(
            applyRequest(() -> sysidSwerveRotationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(m_driveRotationRoutine.dynamic(Direction.kReverse).withTimeout(2))
        .andThen(
            applyRequest(() -> sysidSwerveRotationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .withName("Swerve::SysIdDriveRotation");
  }

  private Timer slipCurrentTimer = new Timer();

  public void setCoastMode() {

    Modules[0].setCoastMode();
    Modules[1].setCoastMode();
    Modules[2].setCoastMode();
    Modules[3].setCoastMode();
  }

  public void setBrakeMode() {
    Modules[0].setBrakeMode();
    Modules[1].setBrakeMode();
    Modules[2].setBrakeMode();
    Modules[3].setBrakeMode();
  }

  public Command slipCurrent() {
    return runOnce(() -> slipCurrentTimer.restart())
        .andThen(
            applyRequest(
                () -> {
                  Logger.recordOutput(
                      "SlipCurrentAmps", sysidTranslationRequest.AmpsToApply.in(Amps));
                  return sysidTranslationRequest.withAmps(Amps.of(slipCurrentTimer.get() * -5));
                }))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Swerve::SlipCurrentAnalysis");
  }

  public final Trigger facingAngleRequestRunning =
      new Trigger(
          () ->
              m_requestToApply == fieldCentricFacingAngle
                  || m_requestToApply == robotCentricFacingAngle
                  || m_requestToApply == fieldCentricFacingPose
                  || m_requestToApply == robotCentricFacingPose);
  public final Trigger stopTurningRequestRunning =
      new Trigger(
          () ->
              m_requestToApply == fieldCentricStopRotation
                  || m_requestToApply == robotCentricStopRotation);

  public void seedStopTurningState() {
    var state = getState();

    stopTurningProfileFollower.setCurrentState(
        state.Pose.getRotation().toRotation2d(), state.speeds.omegaRadiansPerSecond);
  }

  public void seedFacingAngleState() {
    var state = getState();

    facingAngleProfileFollower.setCurrentState(
        state.Pose.getRotation().toRotation2d(), state.speeds.omegaRadiansPerSecond);
  }

  public Rotation2d getFacingError() {
    if (m_requestToApply != fieldCentricFacingPoint) {
      return Rotation2d.fromDegrees(180); // maximum error if we're not running this
    }
    // var state = getState();
    // var change = fieldCentricFacingPoint.target.minus(state.Pose.toPose2d().getTranslation());
    // var heading = change.getAngle().getDegrees();

    return Rotation2d.fromRadians(fieldCentricFacingPoint.HeadingController.getPositionError());
    // return Rotation2d.fromDegrees(heading);
  }

  public class IOInputs implements LoggableInputs {
    SwerveDriveState SwerveDriveState = new SwerveDriveState();
    VisionModule.VisionEstimate VisionEstimate =
        VisionModule.VisionEstimate.builder()
            .timestamp(0)
            .estimate(new Pose3d())
            .stdDevs(new Vector<>(Nat.N6()))
            .build();
    ChoreoTrajectoryState TrajectoryState = new ChoreoTrajectoryState(0, 0, 0, 0, 0, 0, 0);
    SwerveControlRequestParameters ControlParameters = getControlParameters();

    @Override
    public void toLog(LogTable table) {
      table.put("SwerveDriveState/FailedDaqs", SwerveDriveState.FailedDaqs);
      table.put("SwerveDriveState/OdometryPeriod", SwerveDriveState.OdometryPeriod);
      table.put("SwerveDriveState/SuccessfulDaqs", SwerveDriveState.SuccessfulDaqs);
      table.put("SwerveDriveState/ModuleStates", SwerveDriveState.ModuleStates);
      table.put("SwerveDriveState/ModuleTargets", SwerveDriveState.ModuleTargets);
      table.put("SwerveDriveState/Pose", SwerveDriveState.Pose);
      table.put("SwerveDriveState/speeds", SwerveDriveState.speeds);
      table.put("VisionEstimate/timestamp", VisionEstimate.getTimestamp());
      table.put("VisionEstimate/Estimate", VisionEstimate.getEstimate());
      table.put("VisionEstimate/Stdevs", VisionEstimate.getStdDevs().getData());
      table.put("TrajectoryState/timestamp", TrajectoryState.timestamp);
      table.put("TrajectoryState/Pose", TrajectoryState.getPose());
      table.put("TrajectoryState/speeds", TrajectoryState.getChassisSpeeds());
      table.put("ControlParameters/timestamp", ControlParameters.timestamp);
      table.put("ControlParameters/updatePeriod", ControlParameters.updatePeriod);
      table.put("ControlParameters/currentChassisSpeed", ControlParameters.currentChassisSpeed);
      table.put("ControlParameters/currentPose", ControlParameters.currentPose);
      table.put(
          "ControlParameters/operatorForwardDirection", ControlParameters.operatorForwardDirection);
      table.put("Request/FieldCentricFacingPoint/Speeds", fieldCentricFacingPoint.Speeds);
      table.put("Request/FieldCentricFacingPoint/Target", fieldCentricFacingPoint.target);
    }

    @Override
    public void fromLog(LogTable table) {
      SwerveDriveState.FailedDaqs =
          table.get("SwerveDriveState/FailedDaqs", SwerveDriveState.FailedDaqs);
      SwerveDriveState.OdometryPeriod =
          table.get("SwerveDriveState/OdometryPeriod", SwerveDriveState.OdometryPeriod);
      SwerveDriveState.SuccessfulDaqs =
          table.get("SwerveDriveState/SuccessfulDaqs", SwerveDriveState.SuccessfulDaqs);
      SwerveDriveState.ModuleStates =
          table.get("SwerveDriveState/ModuleStates", SwerveDriveState.ModuleStates);
      SwerveDriveState.ModuleTargets =
          table.get("SwerveDriveState/ModuleTargets", SwerveDriveState.ModuleTargets);
      SwerveDriveState.Pose = table.get("SwerveDriveState/Pose", SwerveDriveState.Pose);
      SwerveDriveState.speeds = table.get("SwerveDriveState/speeds", SwerveDriveState.speeds);
      VisionEstimate =
          VisionModule.VisionEstimate.builder()
              .timestamp(table.get("VisionEstimate/timestamp", VisionEstimate.getTimestamp()))
              .estimate(table.get("VisionEstimate/Estimate", VisionEstimate.getEstimate()))
              .stdDevs(
                  new Vector<N6>(
                      new SimpleMatrix(
                          table.get(
                              "VisionEstimate/Stdevs", VisionEstimate.getStdDevs().getData()))))
              .build();
      var timestamp = table.get("TrajectoryState/timestamp", TrajectoryState.timestamp);
      var pose = table.get("TrajectoryState/Pose", TrajectoryState.getPose());
      var speeds = table.get("TrajectoryState/speeds", TrajectoryState.getChassisSpeeds());
      TrajectoryState =
          new ChoreoTrajectoryState(
              timestamp,
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians(),
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond);

      ControlParameters.timestamp =
          table.get("ControlParameters/timestamp", ControlParameters.timestamp);
      ControlParameters.updatePeriod =
          table.get("ControlParameters/updatePeriod", ControlParameters.updatePeriod);
      ControlParameters.currentChassisSpeed =
          table.get("ControlParameters/currentChassisSpeed", ControlParameters.currentChassisSpeed);
      ControlParameters.currentPose =
          table.get("ControlParameters/currentPose", ControlParameters.currentPose);
      ControlParameters.operatorForwardDirection =
          table.get(
              "ControlParameters/operatorForwardDirection",
              ControlParameters.operatorForwardDirection);

      fieldCentricFacingPoint.Speeds =
          table.get("Request/FieldCentricFacingPoint/Speeds", fieldCentricFacingPoint.Speeds);
      fieldCentricFacingPoint.target =
          table.get("Request/FieldCentricFacingPoint/Target", fieldCentricFacingPoint.target);
    }
  }
}
