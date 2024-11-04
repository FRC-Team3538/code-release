package com.robojackets;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.SignalLogger;
import com.robojackets.constants.FieldConstants;
import com.robojackets.constants.TunerConstants;
import com.robojackets.lib.AllianceFlipUtil;
import com.robojackets.oi.CSSelector;
import com.robojackets.oi.ControlScheme;
import com.robojackets.subsystems.Climber;
import com.robojackets.subsystems.Director;
import com.robojackets.subsystems.Elevator;
import com.robojackets.subsystems.Intake;
import com.robojackets.subsystems.NoteHandler;
import com.robojackets.subsystems.Shooter;
import com.robojackets.subsystems.ShooterPivot;
import com.robojackets.subsystems.swerve.CTRESwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public ControlScheme controls;

  public CANdle candle = new CANdle(33);

  public final CTRESwerveDrive swerve = TunerConstants.getInstance().DriveTrain;
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Director director = new Director();
  public final NoteHandler notehandler = new NoteHandler();
  public final ShooterPivot pivot = new ShooterPivot();

  public double previouspos = visiondistance();

  public RobotContainer() {
    SignalLogger.enableAutoLogging(true);
    SignalLogger.start();

    controls = CSSelector.findMainControls();
  }

  public void ConfigureControls() {
    swerve.setDefaultCommand(swerve.TeleopDrive(controls));

    // new Trigger(controls::sysidSteer).whileTrue(swerve.sysidSteer());
    // new
    // Trigger(controls::sysidDriveTranslation).whileTrue(swerve.sysidDriveTranslation());
    // new
    // Trigger(controls::sysidDriveRotation).whileTrue(swerve.sysidDriveRotation());
    // new Trigger(controls::slipCurrent).whileTrue(swerve.slipCurrent());
    new Trigger(controls::getRightPaddle)
        .whileTrue(
            swerve.FieldCentricFacingPoint(
                controls,
                () -> {
                  if (visiondistance() > 7.5) {
                    return AllianceFlipUtil.apply(FieldConstants.feedTarget);
                  } else {
                    return AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
                        .toTranslation2d();
                  }
                }));

    new Trigger(controls::getLeftPaddle)
        .whileTrue(swerve.FieldCentricFacingAngle(controls, Rotation2d.fromRotations(0.25)));

    new Trigger(controls::getR1Driver)
        .whileTrue(
            swerve.FieldCentricFacingPoint(
                controls,
                () -> {
                  if (visiondistance() > 13) {
                    return AllianceFlipUtil.apply(FieldConstants.sourceFeedTarget);
                  } else if (visiondistance() > 7.5) {
                    return AllianceFlipUtil.apply(FieldConstants.feedTarget);
                  } else {
                    return AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
                        .toTranslation2d();
                  }
                }));
  }

  public double visiondistance() {
    var swerveState = swerve.getState();
    var target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);

    var offset = target.toTranslation2d().minus(swerveState.Pose.toPose2d().getTranslation());
    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            swerveState.speeds, swerveState.Pose.getRotation().toRotation2d());

    var z = target.getZ() - Units.inchesToMeters(9.0);
    var d = offset.getNorm();

    // var changedist = visiondistance() - previouspos;
    // var scaler = 1.5 * changedist;
    return d;
  }

  public double addingvalue() {
    var changedist = visiondistance() - previouspos;
    var scaler = 1.0 * changedist;
    return scaler;
  }

  public double previouspos() {
    return previouspos;
  }

  public double anglecalcuation() {
    // UNTESTED: I saw someone on CD post this equation for shooting while moving no clue if it
    // actually works
    var swerveState = swerve.getState();
    var target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);

    var offset = target.toTranslation2d().minus(swerveState.Pose.toPose2d().getTranslation());
    var robotdistance = offset.getNorm();

    var heightdifference = target.getZ() - Units.inchesToMeters(9.0);
    var heightdifferenceroot = Math.sqrt(heightdifference);

    var shooterangle = Math.atan((2 * robotdistance) * heightdifference * heightdifferenceroot);
    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            swerveState.speeds, swerveState.Pose.getRotation().toRotation2d());

    return shooterangle;
  }
}
