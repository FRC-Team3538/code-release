package com.robojackets.subsystems;

import static com.robojackets.oi.HeadingTarget.NONE;
import static com.robojackets.oi.HeadingTarget.PASS;
import static com.robojackets.oi.HeadingTarget.SPEAKER;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.robojackets.constants.FieldConstants;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentric;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentricFacingAngle;
import com.robojackets.ctre.RJSwerveRequest.RJFieldCentricFacingPoint;
import com.robojackets.ctre.RJSwerveRequest.RJFollowTrajectory;
import com.robojackets.ctre.RJSwerveRequest.RJHoldPose;
import com.robojackets.ctre.RJSwerveRequest.RJRobotCentric;
import com.robojackets.ctre.RJSwerveRequest.RJRobotCentricFacingAngle;
import com.robojackets.ctre.RJSwerveRequest.RJRobotCentricFacingPoint;
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
import com.robojackets.ctre.SwerveRequest.SwerveDriveBrake;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveRotation;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveSteerGains;
import com.robojackets.ctre.SwerveRequest.SysIdSwerveTranslation;
import com.robojackets.lib.AllianceFlipUtil;
import com.robojackets.lib.RJLog;
import com.robojackets.lib.trajectory.MotionProfile;
import com.robojackets.lib.trajectory.RotationMotionProfileFollower;
import com.robojackets.lib.trajectory.TrapezoidCurve;
import com.robojackets.oi.ControlMode;
import com.robojackets.oi.DriverControls;
import com.robojackets.oi.HeadingTarget;
import com.robojackets.subsystems.vision.VisionModule.VisionEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;

public class Drive extends SwerveDrivetrain implements Subsystem {

  private Alliance alliance = Alliance.Blue;
  private ChoreoTrajectoryState trajectoryState;

  private final TrapezoidCurve.Constraints rotationConstraints =
      new TrapezoidCurve.Constraints(8 * Math.PI)
          .withMaxVelocity(2 * Math.PI); // accelerate at 2 pi rad/s/s, set max velocity to pi rad/s

  private final RotationMotionProfileFollower<MotionProfile<TrapezoidCurve>>
      stopTurningProfileFollower = rotationConstraints.asVelocityMotionProfile().rotationFollower();
  private final RotationMotionProfileFollower<MotionProfile<TrapezoidCurve>>
      facingAngleProfileFollower =
          rotationConstraints.asFullStateMotionProfile().rotationFollower();

  private final DriveRequestType driveRequestType =
      Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity;
  private final SteerRequestType steerRequestType = SteerRequestType.MotionMagic;

  private final SwerveDriveBrake brake =
      new SwerveDriveBrake()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);

  private final RJFieldCentric fieldCentric =
      new RJFieldCentric()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private final RJRobotCentric robotCentric =
      new RJRobotCentric()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);

  private final RJFieldCentricFacingAngle fieldCentricStopRotation =
      new RJFieldCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(stopTurningProfileFollower)
          .withRotationalDeadband(0.1);
  private final RJRobotCentricFacingAngle robotCentricStopRotation =
      new RJRobotCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(stopTurningProfileFollower)
          .withRotationalDeadband(0.1);

  private final RJFieldCentricFacingAngle fieldCentricFacingAngle =
      new RJFieldCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(facingAngleProfileFollower);
  private final RJRobotCentricFacingAngle robotCentricFacingAngle =
      new RJRobotCentricFacingAngle()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withRotationProfileFollower(facingAngleProfileFollower);

  private final RJFieldCentricFacingPoint fieldCentricFacingPoint =
      new RJFieldCentricFacingPoint()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);
  private final RJRobotCentricFacingPoint robotCentricFacingPoint =
      new RJRobotCentricFacingPoint()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);

  private final RJFollowTrajectory followTrajectory =
      new RJFollowTrajectory()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType)
          .withTelemetryFunc(state -> trajectoryState = state);

  private final RJHoldPose holdPose =
      new RJHoldPose()
          .withDriveRequestType(driveRequestType)
          .withSteerRequestType(steerRequestType);

  private final SwerveSteerImpulseResponse steerImpulseResponse = new SwerveSteerImpulseResponse();
  private final SysIdSwerveTranslationAmps sysidTranslationRequest =
      new SysIdSwerveTranslationAmps();
  private final SysIdSwerveRotationAmps sysidRotationRequest = new SysIdSwerveRotationAmps();
  private final SysIdSwerveSteerGainsAmps sysidSteerGainsRequest = new SysIdSwerveSteerGainsAmps();
  private final SysIdSwerveSteerGains sysidSteerGainsStopRequest = new SysIdSwerveSteerGains();
  private final SysIdSwerveTranslation sysidSwerveTranslationStopRequest =
      new SysIdSwerveTranslation();
  private final SysIdSwerveRotation sysidSwerveRotationStopRequest = new SysIdSwerveRotation();

  private final SysIdRoutine m_driveTranslationRoutine =
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

  private final SysIdRoutine m_driveTranslationAmpsRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second),
              Volts.of(30),
              Seconds.of(5),
              state -> SignalLogger.writeString("drive-translation-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidTranslationRequest.withAmps(Amps.of(volts.in(Volts)))),
              null,
              this));

  private final SysIdRoutine m_driveRotationRoutine =
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

  private final SysIdRoutine m_steerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(5),
              Seconds.of(5),
              state -> SignalLogger.writeString("steer-state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> this.setControl(sysidSteerGainsStopRequest.withVolts(volts)), null, this));

  private HeadingTarget sourcePassTarget = HeadingTarget.SOURCE_TARGET_A;
  private HeadingTarget headingTarget = HeadingTarget.NONE;

  public void setAlliance(Alliance alliance) {
    setOperatorPerspectiveForward(
        AllianceFlipUtil.mirror(getOperatorPerspectiveForward(), this.alliance.equals(alliance)));
    this.alliance = alliance;
  }

  private double lastTime = 0;

  public void recordTelemetry(SwerveDriveState state) {
    RJLog.log("Swerve/State", state);

    if (Utils.isSimulation()) {
      var currentTime = Utils.getCurrentTimeSeconds();
      updateSimState(currentTime - lastTime, 12);
      lastTime = currentTime;
    }
  }

  public Drive(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... moduleConstants) {
    super(driveConstants, 250, moduleConstants);

    var mountOrientation = new MountPoseConfigs();
    mountOrientation.MountPosePitch = -0.18755798041820526;
    mountOrientation.MountPoseRoll = -2.8393754959106445;
    mountOrientation.MountPoseYaw = -87.38851165771484;

    m_pigeon2.getConfigurator().apply(mountOrientation);

    registerTelemetry(this::recordTelemetry);

    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    robotCentricFacingAngle.HeadingController.setP(4);
    fieldCentricFacingAngle.HeadingController.setP(4);

    robotCentricFacingAngle.HeadingController.setD(0.1);
    fieldCentricFacingAngle.HeadingController.setD(0.1);

    robotCentricStopRotation.HeadingController.setP(4);
    fieldCentricStopRotation.HeadingController.setP(4);

    robotCentricStopRotation.HeadingController.setD(0.1);
    fieldCentricStopRotation.HeadingController.setD(0.1);

    fieldCentricFacingPoint.HeadingController.setP(4);
    fieldCentricFacingPoint.HeadingController.setD(0.1);

    followTrajectory.XController.setP(5);
    followTrajectory.YController.setP(5);
    followTrajectory.HeadingController.setP(4);

    holdPose.XController.setP(5);
    holdPose.YController.setP(5);
    holdPose.HeadingController.setP(4);

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

    facingAngleRequestRunning.whileFalse(
        Commands.run(() -> seedFacingAngleState()).withName("Drive::seedFacingAngleState"));
    stopTurningRequestRunning.whileFalse(
        Commands.run(() -> seedStopTurningState()).withName("Drive::seedStopTurningState"));
  }

  public void engageBrakes() {
    for (var module : Modules) {
      module.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void disengageBrakes() {
    for (var module : Modules) {
      module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public void setHeadingTarget(HeadingTarget target) {
    this.headingTarget = target;
  }

  public Command setHeadingTargetCommand(HeadingTarget target) {
    return Commands.runOnce(() -> setHeadingTarget(target))
        .withName(String.format("Drive::SetHeadingTarget(%s)", target));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command fieldCentricCommand(Supplier<ChassisSpeeds> fieldCentricInput) {
    return applyRequest(() -> fieldCentric.withSpeeds(fieldCentricInput.get()))
        .withName("Drive::FieldCentric");
  }

  public Command robotCentricCommand(Supplier<ChassisSpeeds> robotCentricInput) {
    return applyRequest(() -> robotCentric.withSpeeds(robotCentricInput.get()))
        .withName("Drive::RobotCentric");
  }

  public Command fieldCentricFacingAngleCommand(
      Supplier<ChassisSpeeds> fieldCentricInput, Supplier<Rotation2d> heading) {
    return applyRequest(
            () ->
                fieldCentricFacingAngle
                    .withSpeeds(fieldCentricInput.get())
                    .withTargetDirection(heading.get()))
        .withName("Drive::FieldCentricFacingAngle");
  }

  public Command robotCentricFacingAngleCommand(
      Supplier<ChassisSpeeds> robotCentricInput, Supplier<Rotation2d> heading) {
    return applyRequest(
            () ->
                robotCentricFacingAngle
                    .withSpeeds(robotCentricInput.get())
                    .withTargetDirection(heading.get()))
        .withName("Drive::RobotCentricFacingAngle");
  }

  public Command fieldCentricFacingTargetCommand(
      Supplier<ChassisSpeeds> fieldCentricInput, Supplier<Translation2d> target) {
    return applyRequest(
            () ->
                fieldCentricFacingPoint
                    .withSpeeds(fieldCentricInput.get())
                    .withTargetPoint(target.get()))
        .withName("Drive::FieldCentricFacingTarget");
  }

  public Command robotCentricFacingTargetCommand(
      Supplier<ChassisSpeeds> robotCentricInput, Supplier<Translation2d> target) {
    return applyRequest(
            () ->
                robotCentricFacingPoint
                    .withSpeeds(robotCentricInput.get())
                    .withTargetPoint(target.get()))
        .withName("Drive::RobotCentricFacingTarget");
  }

  public Command FollowPath(ChoreoTrajectory traj) {
    return Commands.runOnce(() -> followTrajectory.stopwatch.restart())
        .andThen(applyRequest(() -> followTrajectory.withTrajectory(traj)))
        .withTimeout(traj.getTotalTime())
        .andThen(LockWheels())
        .withName("Drive::FollowPath");
  }

  public Command ImpulseResponse() {
    return applyRequest(() -> steerImpulseResponse).withName("Drive::SteerImpulseResponse");
  }

  public Command HoldPose(Pose2d pose) {
    return applyRequest(() -> holdPose.withTargetPose(pose)).withName("Drive::HoldPose");
  }

  public Command LockWheels() {
    return applyRequest(() -> brake).withName("Drive::LockWheels");
  }

  public Command teleopCommand(DriverControls driver, Supplier<ControlMode> controlModeSupplier) {
    return applyRequest(() -> teleop(driver, controlModeSupplier.get())).withName("Drive::Teleop");
  }

  public SwerveRequest teleop(DriverControls driver, ControlMode controlMode) {
    switch (controlMode) {
      case CLEANUP_MODE:
      case SPEAKER_MODE:
        if (driver.aim()) {
          return headingSnap(driver, controlMode, SPEAKER);
        }
        break;
      case PASS_MODE:
        if (driver.aim()) {
          return headingSnap(driver, controlMode, PASS);
        }
        break;
      case SOURCE_PASS_MODE:
        if (!driver.aim()) {
          return headingSnap(driver, controlMode, sourcePassTarget);
        }
        break;
    }

    if (headingTarget != NONE) {
      return headingSnap(driver, controlMode, headingTarget);
    }

    return manualRotation(driver, controlMode);
  }

  public SwerveRequest manualRotation(DriverControls driver, ControlMode controlMode) {
    if (driver.shouldUseRobotCentric()) {
      return robotCentric.withSpeeds(driver.robotCentric(controlMode));
    }

    return fieldCentric.withSpeeds(driver.fieldCentric(controlMode));
  }

  public SwerveRequest headingSnap(
      DriverControls driver, ControlMode controlMode, HeadingTarget target) {
    switch (target) {
      case INTAKE:
      case NORTH:
      case SOUTH:
      case EAST:
      case WEST:
        return faceHeading(driver, controlMode, FieldConstants.getHeading(alliance, target));

      case SPEAKER:
      case PASS:
      case SOURCE_TARGET_A:
      case SOURCE_TARGET_B:
        return faceTarget(driver, controlMode, FieldConstants.getTarget(alliance, target));

      case NONE:
      default:
        break;
    }

    return manualRotation(driver, controlMode);
  }

  public SwerveRequest faceHeading(
      DriverControls driver, ControlMode controlMode, Rotation2d target) {
    if (driver.shouldUseRobotCentric()) {
      return robotCentricFacingAngle
          .withSpeeds(driver.robotCentric(controlMode))
          .withTargetDirection(target);
    }

    return fieldCentricFacingAngle
        .withSpeeds(driver.fieldCentric(controlMode))
        .withTargetDirection(target);
  }

  public SwerveRequest faceTarget(
      DriverControls driver, ControlMode controlMode, Translation2d target) {
    if (driver.shouldUseRobotCentric()) {
      return robotCentricFacingPoint
          .withSpeeds(driver.robotCentric(controlMode))
          .withTargetPoint(target);
    }

    return fieldCentricFacingPoint
        .withSpeeds(driver.fieldCentric(controlMode))
        .withTargetPoint(target);
  }

  public final Trigger facingAngleRequestRunning =
      new Trigger(
          () ->
              m_requestToApply == fieldCentricFacingAngle
                  || m_requestToApply == robotCentricFacingAngle);
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

  public Measure<Distance> distanceTo(Translation2d point) {
    return Meters.of(getPose().toPose2d().getTranslation().getDistance(point));
  }

  public void resetYaw() {
    setOperatorForwardToCurrentDirection();
  }

  public void setSourcePassTarget(HeadingTarget target) {
    sourcePassTarget = target;
  }

  public boolean isReady(Measure<Angle> tolerance) {
    if (m_requestToApply == fieldCentric) {
      return true;
    } else if (m_requestToApply == robotCentric) {
      return true;
    } else if (m_requestToApply == brake) {
      return true;
    } else if (m_requestToApply == fieldCentricStopRotation) {
      return fieldCentricStopRotation.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == fieldCentricFacingAngle) {
      return fieldCentricFacingAngle.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == robotCentricStopRotation) {
      return robotCentricStopRotation.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == robotCentricFacingAngle) {
      return robotCentricFacingAngle.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == followTrajectory) {
      return followTrajectory.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == holdPose) {
      return holdPose.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == fieldCentricFacingPoint) {
      return fieldCentricFacingPoint.HeadingController.getPositionError() < tolerance.in(Radians);
    } else if (m_requestToApply == robotCentricFacingPoint) {
      return robotCentricFacingPoint.HeadingController.getPositionError() < tolerance.in(Radians);
    }

    return false;
  }

  public void addVisionEstimate(VisionEstimate estimate) {
    addVisionMeasurement(estimate.getEstimate(), estimate.getTimestamp(), estimate.getStdDevs());
  }

  public Pose3d getPose() {
    return getState().Pose;
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
    return m_driveTranslationAmpsRoutine
        .quasistatic(Direction.kForward)
        .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 3)
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 3))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .dynamic(Direction.kForward)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 3))
        .andThen(
            applyRequest(() -> sysidSwerveTranslationStopRequest.withVolts(Volts.of(0)))
                .withTimeout(1))
        .andThen(
            m_driveTranslationRoutine
                .dynamic(Direction.kReverse)
                .until(() -> Math.abs(getState().speeds.vxMetersPerSecond) > 3))
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

  public Command slipCurrent() {
    return runOnce(() -> slipCurrentTimer.restart())
        .andThen(
            applyRequest(
                () -> {
                  RJLog.log("SlipCurrentAmps", sysidTranslationRequest.AmpsToApply);
                  return sysidTranslationRequest.withAmps(Amps.of(slipCurrentTimer.get() * -5));
                }))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Swerve::SlipCurrentAnalysis");
  }

  @Override
  public void periodic() {
    if (trajectoryState != null) {
      RJLog.log("Drive/TrajectoryState", trajectoryState);
    }
    RJLog.log("Drive/HeadingTarget", headingTarget);
  }

  public void resetPose(Pose2d pose) {
    seedFieldRelative(new Pose3d(pose));
  }
}
