package com.robojackets.ctre;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Amps;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.robojackets.ctre.SwerveModule.DriveRequestType;
import com.robojackets.lib.controller.CTREHolonomicController;
import com.robojackets.lib.trajectory.RotationMotionProfileFollower;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Function;

public class RJSwerveRequest {
  /**
   * Drives the swerve drivetrain in a field-centric manner.
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the field, and the rate at which their robot should rotate about the center of the
   * robot.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the robot would drive
   * northward at 5 m/s and turn counterclockwise at 0.5 rad/s.
   */
  public static class RJFieldCentric implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    /** The perspective to use when determining which direction is forward. */
    public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

    /** The last applied state in case we don't have anything to drive. */
    protected SwerveModuleState[] m_lastAppliedState = null;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var toApply =
          ForwardReference.adjustChassisSpeeds(Speeds, parameters.operatorForwardDirection);

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(toApply, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public RJFieldCentric withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJFieldCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJFieldCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJFieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJFieldCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJFieldCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired direction
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the field, and the direction the robot should be facing.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
   * northward at 5 m/s and turn clockwise to a target of 180 degrees.
   *
   * <p>This control request is especially useful for autonomous control, where the robot should be
   * facing a changing direction throughout the motion.
   */
  public static class RJFieldCentricFacingAngle implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /**
     * The desired direction to face. 0 Degrees is defined as in the direction of the X axis. As a
     * result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     */
    public RotationMotionProfileFollower<?> RotationProfileFollower;

    public Function<SwerveControlRequestParameters, Rotation2d> TargetDirection =
        state -> new Rotation2d();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second.
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** The perspective to use when determining which direction is forward. */
    public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

    // TODO: consider prioritizing following rotation motion profile such that you slow down in
    // order to adhere to it
    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var toApply =
          ForwardReference.adjustChassisSpeeds(Speeds, parameters.operatorForwardDirection);

      this.RotationProfileFollower.setGoalState(TargetDirection.apply(parameters), 0);

      var state = RotationProfileFollower.update(parameters.updatePeriod);

      toApply.omegaRadiansPerSecond = state.velocity;

      toApply.omegaRadiansPerSecond +=
          HeadingController.calculate(
              parameters.currentPose.getRotation().getRadians(),
              state.position,
              parameters.timestamp);

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(toApply, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain. Only uses translational velocities.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public RJFieldCentricFacingAngle withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJFieldCentricFacingAngle withRotationProfileFollower(
        RotationMotionProfileFollower<?> rotationProfileFollower) {
      this.RotationProfileFollower = rotationProfileFollower;
      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJFieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
      this.TargetDirection = state -> targetDirection;

      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJFieldCentricFacingAngle withTargetDirection(
        Function<SwerveControlRequestParameters, Rotation2d> targetDirection) {
      this.TargetDirection = targetDirection;

      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJFieldCentricFacingAngle withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJFieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJFieldCentricFacingAngle withDriveRequestType(
        SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJFieldCentricFacingAngle withSteerRequestType(
        SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a robot-centric manner.
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the robot itself, and the rate at which their robot should rotate about the center of
   * the robot.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the robot would drive
   * eastward at 5 m/s and turn counterclockwise at 0.5 rad/s.
   */
  public static class RJRobotCentric implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var toApply = Speeds;
      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      var states = parameters.kinematics.toSwerveModuleStates(toApply, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public RJRobotCentric withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJRobotCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJRobotCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJRobotCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJRobotCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJRobotCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a robo5-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired direction
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
   * northward at 5 m/s and turn clockwise to a target of 180 degrees.
   *
   * <p>This control request is especially useful for autonomous control, where the robot should be
   * facing a changing direction throughout the motion.
   */
  public static class RJRobotCentricFacingAngle implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /**
     * The desired direction to face. 0 Degrees is defined as in the direction of the X axis. As a
     * result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     */
    public RotationMotionProfileFollower<?> RotationProfileFollower;

    public Function<SwerveControlRequestParameters, Rotation2d> TargetDirection =
        state -> new Rotation2d();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second.
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var toApply = Speeds;

      this.RotationProfileFollower.setGoalState(TargetDirection.apply(parameters), 0);

      var state = RotationProfileFollower.update(parameters.updatePeriod);

      toApply.omegaRadiansPerSecond = state.velocity;

      toApply.omegaRadiansPerSecond +=
          HeadingController.calculate(
              parameters.currentPose.getRotation().getRadians(),
              state.position,
              parameters.timestamp);

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(toApply, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain. Only uses translational velocities.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public RJRobotCentricFacingAngle withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJRobotCentricFacingAngle withRotationProfileFollower(
        RotationMotionProfileFollower<?> rotationProfileFollower) {
      this.RotationProfileFollower = rotationProfileFollower;
      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJRobotCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
      this.TargetDirection = state -> targetDirection;

      return this;
    }

    /**
     * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
     *
     * @param targetDirection Desired direction to face
     * @return this request
     */
    public RJRobotCentricFacingAngle withTargetDirection(
        Function<SwerveControlRequestParameters, Rotation2d> targetDirection) {
      this.TargetDirection = targetDirection;

      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJRobotCentricFacingAngle withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJRobotCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJRobotCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJRobotCentricFacingAngle withDriveRequestType(
        SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJRobotCentricFacingAngle withSteerRequestType(
        SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class RJFollowTrajectory implements SwerveRequest {
    public ChoreoTrajectory Trajectory = new ChoreoTrajectory();
    public Timer stopwatch = new Timer();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public CTREHolonomicController Controller =
        new CTREHolonomicController(XController, YController, HeadingController);

    public Consumer<ChoreoTrajectoryState> telemetryFunc;

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var state = Trajectory.sample(stopwatch.get());

      if (telemetryFunc != null) {
        telemetryFunc.accept(state);
      }

      ChassisSpeeds toApply =
          Controller.calculate(
              parameters.currentPose,
              state.getPose(),
              state.getChassisSpeeds(),
              parameters.timestamp);

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds = ChassisSpeeds.discretize(toApply, parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    public RJFollowTrajectory withTrajectory(ChoreoTrajectory traj) {
      this.Trajectory = traj;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJFollowTrajectory withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJFollowTrajectory withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJFollowTrajectory withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJFollowTrajectory withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJFollowTrajectory withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }

    public boolean isFinished() {
      return this.stopwatch.get() >= Trajectory.getTotalTime();
    }

    public RJFollowTrajectory withTelemetryFunc(Consumer<ChoreoTrajectoryState> telemetry) {
      this.telemetryFunc = telemetry;

      return this;
    }
  }

  public static class RJHoldPose implements SwerveRequest {
    public Pose2d targetPose = new Pose2d();

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public CTREHolonomicController Controller =
        new CTREHolonomicController(XController, YController, HeadingController);

    private final ChassisSpeeds dontmove = new ChassisSpeeds();

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

      ChassisSpeeds toApply =
          Controller.calculate(parameters.currentPose, targetPose, dontmove, parameters.timestamp);

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds = ChassisSpeeds.discretize(toApply, parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    public RJHoldPose withTargetPose(Pose2d pose) {
      this.targetPose = pose;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJHoldPose withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJHoldPose withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJHoldPose withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJHoldPose withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJHoldPose withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  /**
   * SysId-specific SwerveRequest to characterize the translational characteristics of a swerve
   * drivetrain.
   */
  public static class SysIdSwerveTranslationAmps implements SwerveRequest {
    /* Current to apply to drive wheels. This is final to enforce mutating the value */
    public final MutableMeasure<Current> AmpsToApply = mutable(Amps.of(0));

    /* Local reference to a torque current request to drive the motors with */
    public TorqueCurrentFOC m_ampRequest = new TorqueCurrentFOC(0);

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].applyCharacterization(
            Rotation2d.fromDegrees(0), m_ampRequest.withOutput(AmpsToApply.in(Amps)));
      }
      return StatusCode.OK;
    }

    /**
     * Update the current to apply to the drive wheels.
     *
     * @param Amps Current to apply
     * @return this request
     */
    public SysIdSwerveTranslationAmps withAmps(Measure<Current> Amps) {
      AmpsToApply.mut_replace(Amps);
      return this;
    }
  }

  /**
   * SysId-specific SwerveRequest to characterize the rotational characteristics of a swerve
   * drivetrain.
   */
  public static class SysIdSwerveRotationAmps implements SwerveRequest {
    /* Current to apply to drive wheels. This is final to enforce mutating the value */
    public final MutableMeasure<Current> AmpsToApply = mutable(Amps.of(0));

    /* Local reference to a torque current request to drive the motors with */
    public TorqueCurrentFOC m_ampRequest = new TorqueCurrentFOC(0);

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].applyCharacterization(
            parameters.swervePositions[i].getAngle().plus(Rotation2d.fromDegrees(90)),
            m_ampRequest.withOutput(AmpsToApply.in(Amps)));
      }
      return StatusCode.OK;
    }

    /**
     * Update the voltage to apply to the drive wheels.
     *
     * @param Amps Current to apply
     * @return this request
     */
    public SysIdSwerveRotationAmps withAmps(Measure<Current> Amps) {
      AmpsToApply.mut_replace(Amps);
      return this;
    }
  }

  /**
   * SysId-specific SwerveRequest to characterize the steer module characteristics of a swerve
   * drivetrain.
   */
  public static class SysIdSwerveSteerGainsAmps implements SwerveRequest {
    /* Current to apply to drive wheels. This is final to enforce mutating the value */
    public final MutableMeasure<Current> AmpsToApply = mutable(Amps.of(0));

    /* Local reference to a torque current request to drive the motors with */
    public TorqueCurrentFOC m_ampRequest = new TorqueCurrentFOC(0);

    /* Local reference to a torque current request to drive the motors with */
    public VoltageOut m_voltRequest = new VoltageOut(0);

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].getSteerMotor().setControl(m_ampRequest.withOutput(AmpsToApply.in(Amps)));
        modulesToApply[i].getDriveMotor().setControl(m_voltRequest.withOutput(0));
      }
      return StatusCode.OK;
    }

    /**
     * Update the current to apply to the drive wheels.
     *
     * @param Amps Voltage to apply
     * @return this request
     */
    public SysIdSwerveSteerGainsAmps withAmps(Measure<Current> Amps) {
      AmpsToApply.mut_replace(Amps);
      return this;
    }
  }

  /**
   * SysId-specific SwerveRequest to characterize the steer module characteristics of a swerve
   * drivetrain.
   */
  public static class SwerveSteerImpulseResponse implements SwerveRequest {
    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var state = new SwerveModuleState();
      state.speedMetersPerSecond = 0.3;
      var seconds = (int) Timer.getFPGATimestamp();
      if (seconds % 10 < 5) {
        state.angle = Rotation2d.fromDegrees(90);
      } else {
        state.angle = Rotation2d.fromDegrees(0);
      }

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(state, DriveRequestType.OpenLoopVoltage);
      }
      return StatusCode.OK;
    }
  }

  /**
   * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired direction
   *
   * <p>When users use this request, they specify the direction the robot should travel oriented
   * against the field, and the direction the robot should be facing.
   *
   * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
   * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
   * northward at 5 m/s and turn clockwise to a target of 180 degrees.
   *
   * <p>This control request is especially useful for autonomous control, where the robot should be
   * facing a changing direction throughout the motion.
   */
  public static class RJFieldCentricFacingPoint implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /** The desired point to face toward. */
    public Translation2d target;

    /** The allowable deadband of the request. */
    public double Deadband = 0;
    /** The rotational deadband of the request. */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagic;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second.
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** The perspective to use when determining which direction is forward. */
    public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var toApply =
          ForwardReference.adjustChassisSpeeds(Speeds, parameters.operatorForwardDirection);

      var offset = target.minus(parameters.currentPose.getTranslation());

      var heading = offset.getAngle().plus(Rotation2d.fromRotations(0.5));

      var angularVelocity = 0.0;
      var top =
          toApply.vyMetersPerSecond * offset.getX() - toApply.vxMetersPerSecond * offset.getY();
      var bottom = offset.getX() * offset.getX() + offset.getY() * offset.getY();

      if (bottom > 1e-9) {
        angularVelocity = top / bottom;
      }

      toApply.omegaRadiansPerSecond = -angularVelocity;

      var feedback =
          HeadingController.calculate(
              parameters.currentPose.getRotation().getRadians(),
              heading.getRadians(),
              parameters.timestamp);
      toApply.omegaRadiansPerSecond += feedback;

      if (Math.sqrt(
              toApply.vxMetersPerSecond * toApply.vxMetersPerSecond
                  + toApply.vyMetersPerSecond * toApply.vyMetersPerSecond)
          < Deadband) {
        toApply.vxMetersPerSecond = 0;
        toApply.vyMetersPerSecond = 0;
      }
      if (Math.abs(toApply.omegaRadiansPerSecond) < RotationalDeadband) {
        toApply.omegaRadiansPerSecond = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(toApply, parameters.currentPose.getRotation()),
              parameters.updatePeriod);

      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

      // SwerveDriveKinematics.desaturateWheelSpeeds(states,
      // TunerConstants.getInstance().MaxVelocity);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      // Logger.recordOutput("RJFacingPoint/inputSpeeds", Speeds);
      // Logger.recordOutput("RJFacingPoint/feedforward", angularVelocity);
      // Logger.recordOutput("RJFacingPoint/measurement", parameters.currentPose.getRotation());
      // Logger.recordOutput("RJFacingPoint/reference", heading);
      // Logger.recordOutput("RJFacingPoint/feedback", feedback);
      // Logger.recordOutput("RJFacingPoint/toApply", toApply);
      // Logger.recordOutput("RJFacingPoint/outputSpeeds", speeds);
      // Logger.recordOutput("RJFacingPoint/states", states);

      return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain. Only uses translational velocities.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public RJFieldCentricFacingPoint withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    /**
     * Sets the desired position to face toward.
     *
     * @param target Desired point to face toward
     * @return this request
     */
    public RJFieldCentricFacingPoint withTargetPoint(Translation2d target) {
      this.target = target;
      return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public RJFieldCentricFacingPoint withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }
    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public RJFieldCentricFacingPoint withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }
    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate around.
     * @return this request
     */
    public RJFieldCentricFacingPoint withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public RJFieldCentricFacingPoint withDriveRequestType(
        SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }
    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public RJFieldCentricFacingPoint withSteerRequestType(
        SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }
}
