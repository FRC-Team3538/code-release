package com.robojackets;

import static com.robojackets.oi.ControlMode.*;
import static com.robojackets.oi.HeadingTarget.*;
import static com.robojackets.oi.StructureState.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import com.robojackets.constants.FieldConstants;
import com.robojackets.lib.RJLog;
import com.robojackets.oi.ControlModeManager;
import com.robojackets.oi.DriverControls;
import com.robojackets.oi.HeadingTarget;
import com.robojackets.oi.OperatorControls;
import com.robojackets.oi.StructureState;
import com.robojackets.subsystems.Drive;
import com.robojackets.subsystems.Intake;
import com.robojackets.subsystems.Pivot;
import com.robojackets.subsystems.RJTrigger;
import com.robojackets.subsystems.Shooter;
import com.robojackets.subsystems.Shooter.SpinDirection;
import com.robojackets.subsystems.Vision;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private static Vector<N2> setpoint(Measure<Angle> pitch, Measure<Velocity<Angle>> shotVelocity) {
    return VecBuilder.fill(pitch.in(Radians), shotVelocity.in(RevolutionsPerSecond));
  }

  private static Measure<Angle> pitch(Matrix<N2, N1> setpoint) {
    return Radians.of(setpoint.get(0, 0));
  }

  private static Measure<Velocity<Angle>> shotVelocity(Matrix<N2, N1> setpoint) {
    return RevolutionsPerSecond.of(setpoint.get(1, 0));
  }

  private final CommandXboxController driver;
  private final CommandPS4Controller operator;
  private final DriverControls driverControls;
  private final OperatorControls operatorControls;

  public final Drive drive;
  public final Vision vision;
  public final Intake intake;
  public final RJTrigger trigger;
  public final Pivot pivot;
  public final Shooter shooter;
  private final ControlModeManager controlModeManager;

  private Alliance alliance;
  private HeadingTarget sourcePassTarget = HeadingTarget.SOURCE_TARGET_A;

  private final InterpolatingMatrixTreeMap<Double, N2, N1> speakerMap =
      new InterpolatingMatrixTreeMap<>();
  private final InterpolatingMatrixTreeMap<Double, N2, N1> sourcePassMap =
      new InterpolatingMatrixTreeMap<>();
  private final InterpolatingMatrixTreeMap<Double, N2, N1> passMap =
      new InterpolatingMatrixTreeMap<>();

  public RobotContainer(
      Drive drive,
      Vision vision,
      Intake intake,
      Pivot pivot,
      Shooter shooter,
      RJTrigger trigger,
      CommandXboxController driver,
      CommandPS4Controller operator,
      ControlModeManager controlModeManager) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    this.trigger = trigger;
    this.driver = driver;
    this.operator = operator;
    this.controlModeManager = controlModeManager;

    this.driverControls = new DriverControls(driver.getHID());
    this.operatorControls = new OperatorControls(operator.getHID());

    speakerMap.put(1.01, setpoint(Degrees.of(50.0), RPM.of(2000)));
    speakerMap.put(1.58, setpoint(Degrees.of(42.0), RPM.of(2100)));
    speakerMap.put(2.02, setpoint(Degrees.of(36.75), RPM.of(2100)));
    speakerMap.put(2.5, setpoint(Degrees.of(32.0), RPM.of(2200)));
    speakerMap.put(2.86, setpoint(Degrees.of(29.5), RPM.of(2300)));
    speakerMap.put(3.24, setpoint(Degrees.of(27.35), RPM.of(2400)));
    speakerMap.put(3.68, setpoint(Degrees.of(26.0), RPM.of(2500)));
    speakerMap.put(4.16, setpoint(Degrees.of(24.25), RPM.of(2600)));
    speakerMap.put(4.64, setpoint(Degrees.of(21.75), RPM.of(2650)));
    speakerMap.put(5.05, setpoint(Degrees.of(20.3), RPM.of(2700)));

    passMap.put(0.0, setpoint(Degrees.of(37.0), RPM.of(2300)));
    sourcePassMap.put(0.0, setpoint(Degrees.of(0), RPM.of(0)));

    Preferences.initInt("Intake/UnjamSwitchingFrequency", 2);

    Preferences.initDouble("Trigger/Shooter_Tolerance_RPM", 50); // rpm
    Preferences.initDouble("Trigger/Pivot_Tolerance_Deg", 0.5); // degrees
    Preferences.initDouble("Trigger/Drive_Tolerance_Deg", 0.5); // degrees

    configureBehavior();

    universalControls();
    cleanupModeControls();
    speakerModeControls();
    sourcePassModeControls();
    passModeControls();

    if (DriverStation.getAlliance().isPresent()) {
      setAlliance(DriverStation.getAlliance().get());
    }
  }

  public void setSourcePassTarget(HeadingTarget target) {
    drive.setSourcePassTarget(target);
    this.sourcePassTarget = target;
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    drive.setAlliance(alliance);
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public void configureBehavior() {
    disabled()
        .and(
            () ->
                DriverStation.getAlliance().isPresent()
                    && alliance != DriverStation.getAlliance().get())
        .onTrue(
            runOnce(
                    () -> {
                      var alliance = DriverStation.getAlliance().get();
                      setAlliance(alliance);
                    })
                .withName("Robot::SetAlliance"));

    disabled().negate().onTrue(runOnce(() -> drive.engageBrakes()).withName("Drive::EngageBrakes"));
    disabled()
        .whileTrue(
            waitSeconds(5)
                .andThen(() -> drive.disengageBrakes())
                .withName("Drive::DisengageBrakes"));
  }

  public void universalControls() {

    // Control Mode config
    operator.triangle().onTrue(controlModeManager.goToMode(CLEANUP_MODE));
    operator.square().onTrue(controlModeManager.goToMode(SPEAKER_MODE));
    operator.circle().onTrue(controlModeManager.goToMode(SOURCE_PASS_MODE));
    operator.cross().onTrue(controlModeManager.goToMode(PASS_MODE));

    driver.start().onTrue(runOnce(drive::resetYaw).withName("Drive::ResetYaw"));
    driver.back().toggleOnTrue(vision.disableCommand());
    operator
        .pov(90)
        .onTrue(
            runOnce(() -> setSourcePassTarget(SOURCE_TARGET_A))
                .withName("Robot::SetSourcePassTarget(SOURCE_TARGET_A)"));
    operator
        .pov(270)
        .onTrue(
            runOnce(() -> setSourcePassTarget(SOURCE_TARGET_B))
                .withName("Robot::SetSourcePassTarget(SOURCE_TARGET_B)"));

    drive.setDefaultCommand(
        drive.teleopCommand(driverControls, controlModeManager::getCurrentMode));
    new Trigger(driverControls::shouldManuallyRotate).onTrue(drive.setHeadingTargetCommand(NONE));

    // Fire
    driver.rightTrigger().whileTrue(fireCommand(false));

    // Intake
    driver.rightBumper().or(operator.R1().and(operator.options().negate())).whileTrue(intakeNote());

    // Reject
    driver.leftStick().or(operator.L1().and(operator.options().negate())).whileTrue(rejectNote());

    // Unjam
    operator.R1().and(operator.options()).whileTrue(unjamIntake());

    // Manual Pivot
    operator.options().whileTrue(pivot.manualCommand(operatorControls::pivot));
  }

  public void cleanupModeControls() {
    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.y())
        .onTrue(drive.setHeadingTargetCommand(NORTH));
    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.a())
        .onTrue(drive.setHeadingTargetCommand(SOUTH));
    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.b())
        .onTrue(drive.setHeadingTargetCommand(EAST));
    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.x())
        .onTrue(drive.setHeadingTargetCommand(WEST));

    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.leftTrigger())
        .whileTrue(aimCommand(AIM_AT_SPEAKER));
    controlModeManager
        .isMode(CLEANUP_MODE)
        .and(driver.leftTrigger().negate())
        .whileTrue(aimCommand(HOME));
  }

  public void speakerModeControls() {
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.y())
        .onTrue(drive.setHeadingTargetCommand(NORTH));
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.a())
        .onTrue(drive.setHeadingTargetCommand(SOUTH));
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.b())
        .onTrue(drive.setHeadingTargetCommand(EAST));
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.x())
        .onTrue(drive.setHeadingTargetCommand(WEST));

    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.leftTrigger())
        .whileTrue(aimCommand(AIM_AT_SPEAKER));
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.leftTrigger().negate().and(driver.rightTrigger()))
        .whileTrue(aimCommand(FLAT));
    controlModeManager
        .isMode(SPEAKER_MODE)
        .and(driver.leftTrigger().negate().and(driver.rightTrigger().negate()))
        .whileTrue(aimCommand(HOME));
  }

  public void sourcePassModeControls() {
    controlModeManager
        .isMode(SOURCE_PASS_MODE)
        .and(driver.leftTrigger())
        .whileTrue(aimCommand(HOME));
    controlModeManager
        .isMode(SOURCE_PASS_MODE)
        .and(driver.leftTrigger().negate())
        .whileTrue(aimCommand(SHOOT_FROM_SOURCE));
  }

  public void passModeControls() {
    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.y())
        .onTrue(drive.setHeadingTargetCommand(NORTH));
    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.a())
        .onTrue(drive.setHeadingTargetCommand(SOUTH));
    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.b())
        .onTrue(drive.setHeadingTargetCommand(PASS));
    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.x())
        .onTrue(drive.setHeadingTargetCommand(INTAKE));

    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.leftTrigger())
        .whileTrue(aimCommand(AIM_AT_PASS_TARGET));
    controlModeManager
        .isMode(PASS_MODE)
        .and(driver.leftTrigger().negate())
        .whileTrue(aimCommand(HOME));
  }

  private Command intakeNote() {
    return trigger
        .primeCommand()
        .deadlineWith(intake.intakeCommand())
        .withName("Robot::IntakeNote");
  }

  private Command rejectNote() {
    return trigger.poopCommand().alongWith(intake.rejectCommand()).withName("Robot::RejectNote");
  }

  private Command unjamIntake() {
    var switchingFrequency = Preferences.getInt("Intake/UnjamSwitchingFrequency", 2);
    var wait = 1.0 / switchingFrequency;
    return intake
        .intakeCommand()
        .withTimeout(wait)
        .andThen(intake.rejectCommand().withTimeout(wait))
        .repeatedly()
        .withName("Robot::UnjamIntake");
  }

  public Command fireCommand(boolean force) {
    return waitUntil(() -> (force || onTarget()) && shooter.hasNonZeroSetpoint())
        .andThen(trigger.fireCommand())
        .unless(trigger::noteAbsent)
        .withName("Robot::Fire");
  }

  public Command aimCommand(StructureState state) {
    return run(() -> aim(state), pivot, shooter).withName("Robot::Aim");
  }

  public void aim(StructureState state) {
    aim(getSetpoint(state));
  }

  private void aim(Matrix<N2, N1> setpoint) {
    SpinDirection spin;

    switch (alliance) {
      case Blue:
        spin = SpinDirection.Clockwise;
        break;
      case Red:
        spin = SpinDirection.CounterClockwise;
        break;
      default:
        spin = SpinDirection.Neither;
        break;
    }

    pivot.aim(pitch(setpoint));
    shooter.prime(shotVelocity(setpoint), spin);
  }

  private Matrix<N2, N1> getSetpoint(StructureState state) {
    Measure<Distance> distance;
    switch (state) {
      case AIM_AT_PASS_TARGET:
        distance = drive.distanceTo(FieldConstants.getTarget(alliance, HeadingTarget.PASS));
        return passMap.get(distance.in(Meters));

      case AIM_AT_SPEAKER:
        distance = drive.distanceTo(FieldConstants.getTarget(alliance, HeadingTarget.SPEAKER));
        return speakerMap.get(distance.in(Meters));

      case DEFAULT_PASS:
        return setpoint(Degrees.of(37), RPM.of(0));

      case DUMP_PASS:
        return setpoint(Degrees.of(37), RPM.of(2300));

      case FLAT:
        return setpoint(Degrees.of(0), RPM.of(2300));

      case SHOOT_FROM_SOURCE:
        switch (sourcePassTarget) {
          case SOURCE_TARGET_A:
            return setpoint(Degrees.of(37), RPM.of(2300));
          case SOURCE_TARGET_B:
            return setpoint(Degrees.of(37), RPM.of(2100));
          default:
            return getSetpoint(HOME);
        }

      case SHOOT_FROM_SUBWOOFER:
        return setpoint(Degrees.of(50), RPM.of(2000));

      case SHOOT_FROM_TRUSS:
        return setpoint(Degrees.of(32), RPM.of(2100));

      case HOME:
      default:
        return setpoint(Degrees.of(0), RPM.of(0));
    }
  }

  public boolean onTarget() {
    var shooterTolerance = Preferences.getDouble("Trigger/Shooter_Tolerance_RPM", 50); // rpm
    var pivotTolerance = Preferences.getDouble("Trigger/Pivot_Tolerance_Deg", 0.5); // degrees
    var driveTolerance = Preferences.getDouble("Trigger/Drive_Tolerance_Deg", 0.5); // degrees

    var driveReady = drive.isReady(Degrees.of(driveTolerance));
    var shooterReady = shooter.isReady(RPM.of(shooterTolerance));
    var pivotReady = pivot.isReady(Degrees.of(pivotTolerance));

    RJLog.log("Trigger/pivot", pivotReady);
    RJLog.log("Trigger/shooter", shooterReady);
    RJLog.log("Trigger/drive", driveReady);

    return pivotReady && shooterReady && driveReady;
  }

  public void periodic() {
    RJLog.log("Controls/ShouldManuallyRotate", driverControls.shouldManuallyRotate());
    RJLog.log("Controls/rotationInput", driverControls.rotateInternal());
  }
}
