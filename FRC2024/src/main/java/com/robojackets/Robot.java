// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets;

import com.robojackets.Constants.Mode;
import com.robojackets.auto.AutonChooser;
import com.robojackets.lib.AllianceFlipUtil;
import com.robojackets.lib.AllianceMonitor;
import com.robojackets.lib.trajectory.TrajectoryManager;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Timer coaster = new Timer();
  private Timer ampTimer = new Timer();
  private boolean brakemodeoneshot = false;
  private RobotContainer robotContainer;

  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);

  private Command auton;

  String kFJShooterRPMString = "FEED JUICED shooter rpm preset";
  String kFShooterRPMString = "FEED shooter rpm preset";
  String kFShooterAngleString = "FEED shooter angle preset";

  String kSFShooterRPMString = "SOURCE FEED shooter rpm preset";
  String kSFShooterAngleString = "SOURCE FEED shooter angle preset";

  private double kAngleTolerance = 0.6;
  private double kRpmTolerance = 150;
  private double kHeadingTolerance = 1.5;

  private boolean m_manualPivot = false;

  private boolean m_visiontoggle = true;

  private boolean m_indaintake = false;
  private boolean m_indahandler = false;
  private boolean m_stowelevator = false;
  private boolean m_driveramp = false;
  private boolean m_elevatorhold = false;
  private boolean m_elevbackdown = false;
  private boolean m_elevbackdownpivot = false;
  private boolean elevpivot = false;
  private boolean m_pivotdown = false;
  private boolean m_pivotdownmemory = false;

  private boolean m_isintaking = false;
  private boolean m_isshooting = false;

  // public DirectorState m_mode;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    PortForwarder.add(8888, "10.35.38.10", 5800);
    PortForwarder.add(8881, "10.35.38.10", 1181);
    PortForwarder.add(8882, "10.35.38.10", 1182);
    PortForwarder.add(8883, "10.35.38.10", 1183);
    PortForwarder.add(1181, "10.35.38.10", 1181);
    PortForwarder.add(1182, "10.35.38.10", 1182);
    PortForwarder.add(1183, "10.35.38.10", 1183);
    PortForwarder.add(1184, "10.35.38.10", 1184);
    PortForwarder.add(1185, "10.35.38.10", 1185);
    PortForwarder.add(1186, "10.35.38.10", 1186);
    RobotController.setBrownoutVoltage(6.3);
    Threads.setCurrentThreadPriority(true, 1);

    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter());
        // Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    setUseTiming(Constants.getMode() != Mode.REPLAY);
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    robotContainer = new RobotContainer();

    TrajectoryManager.getInstance().LoadTrajectories();
    AutonChooser.Init(robotContainer);

    robotContainer.swerve.tareEverything();
    robotContainer.ConfigureControls();

    // SmartDash
    double FJshooterRPM = SmartDashboard.getNumber(kFJShooterRPMString, 2000); // 2300, 2100
    double FshooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 2000); // 2300, 2100
    double FshooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 25.5); // 21, 24
    SmartDashboard.putNumber(kFJShooterRPMString, FJshooterRPM);
    SmartDashboard.putNumber(kFShooterRPMString, FshooterRPM);
    SmartDashboard.putNumber(kFShooterAngleString, FshooterAngle);
    // SmartDash
    double SFshooterRPM = SmartDashboard.getNumber(kSFShooterRPMString, 2000); // 2300, 2100
    double SFshooterAngle = SmartDashboard.getNumber(kSFShooterAngleString, 30); // 21, 24
    SmartDashboard.putNumber(kSFShooterRPMString, SFshooterRPM);
    SmartDashboard.putNumber(kSFShooterAngleString, SFshooterAngle);
  }

  VisionStatisticsTracker tracker = new VisionStatisticsTracker("Rear ov9281");

  @Override
  public void robotPeriodic() {
    robotContainer.notehandler.SDOutputs();
    robotContainer.shooter.SDOutputs();
    robotContainer.pivot.PivotReadout();
    robotContainer.elevator.ElevatorReadout();
    robotContainer.climber.ClimberReadout();
    robotContainer.intake.IntakeSDOutputs();

    SmartDashboard.putNumber(
        "interpolate pivot value",
        robotContainer.pivot.getInterpolateVals(robotContainer.visiondistance()));
    SmartDashboard.putNumber(
        "interpolate rpm value",
        robotContainer.shooter.getInterpolateVals(robotContainer.visiondistance()));
    SmartDashboard.putNumber("previous pos", robotContainer.previouspos());
    SmartDashboard.putNumber("increasing factor", robotContainer.addingvalue());
    SmartDashboard.putNumber("Vision Distance Away", robotContainer.visiondistance());
    SmartDashboard.putNumber("theoretical angle calculation CD", robotContainer.anglecalcuation());
    SmartDashboard.putNumber(
        "goal heading error", robotContainer.swerve.getFacingError().getDegrees());
    SmartDashboard.putBoolean(
        "drivetrain facing?",
        Math.abs(robotContainer.swerve.getFacingError().getDegrees()) < kHeadingTolerance);

    CommandScheduler.getInstance().run();

    if (robotContainer.controls.getShareButtonPressedDriver()) {
      m_visiontoggle = !m_visiontoggle;
    }

    if (m_visiontoggle) {
      robotContainer.swerve.EnableVision();
    }

    if (!m_visiontoggle) {
      robotContainer.swerve.DisableVision();
    }

    if (robotContainer.controls.resetYaw()) {
      robotContainer.swerve.resetYaw();
    }

    SmartDashboard.putBoolean("Vision On?", m_visiontoggle);
    SmartDashboard.putBoolean("In Intake?", m_indaintake);
    SmartDashboard.putBoolean("in Handler?", m_indahandler);
    SmartDashboard.putBoolean("m_isintaking", m_isintaking);
    SmartDashboard.putBoolean("m_stowelevator", m_stowelevator);

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    Logger.recordOutput("Control Scheme", robotContainer.controls.getName());

    AllianceMonitor.IfAllianceSetThen(
        alliance -> {
          robotContainer.swerve.setOperatorPerspectiveForward(
              AllianceFlipUtil.apply(
                  robotContainer.swerve.getOperatorPerspectiveForward(),
                  Alliance.Red.equals(alliance)));
          robotContainer.shooter.setAlliance(alliance);
        });

    AllianceMonitor.IfAllianceUpdatedThen(
        alliance -> {
          robotContainer.swerve.setOperatorPerspectiveForward(
              AllianceFlipUtil.apply(robotContainer.swerve.getOperatorPerspectiveForward(), true));
          robotContainer.shooter.setAlliance(alliance);
        });

    if (robotContainer.controls.getR2Driver() > 0.05
        || robotContainer.controls.getL2Driver() > 0.05
        || robotContainer.controls.getR2Operator() > 0.05
        || robotContainer.controls.getL2Operator() > 0.05) {
      if (!robotContainer.shooter.NoteNotDetected()
          || robotContainer.notehandler.getTorqueCurrent() > 11.0) {
        robotContainer.candle.setLEDs(0, 255, 0);
      } else {
        robotContainer.candle.setLEDs(255, 200, 0);
      }
    } else if (robotContainer.controls.getR1Driver() || robotContainer.controls.getRightPaddle()) {
      if (robotContainer.shooter.getShooterRPM() == 2300
          && robotContainer.pivot.getAngleCommanded() == 21.0
          && robotContainer.pivot.AtAngle(kAngleTolerance)
          && robotContainer.shooter.AtRPM(kRpmTolerance)
          && Math.abs(robotContainer.swerve.getFacingError().getDegrees()) < kHeadingTolerance) {
        robotContainer.candle.setLEDs(0, 0, 255);
      } else if (robotContainer.pivot.AtAngle(kAngleTolerance)
          && robotContainer.shooter.AtRPM(kRpmTolerance)
          && Math.abs(robotContainer.swerve.getFacingError().getDegrees()) < kHeadingTolerance) {
        robotContainer.candle.setLEDs(0, 255, 0);
      } else if (robotContainer.visiondistance() > 7.5) {
        robotContainer.candle.setLEDs(255, 255, 0);
      } else {
        robotContainer.candle.setLEDs(255, 0, 0);
      }
    } else {
      robotContainer.candle.setLEDs(0, 0, 0, 0, 0, 8);
      robotContainer.candle.setLEDs(0, 255, 100, 0, 9, 60);
    }
  }

  @Override
  public void autonomousInit() {
    // NOTE: If working on elements field, set DS to Blue Alliance

    auton = AutonChooser.getSelectedAuton();
    SmartDashboard.putData("Auton", auton);
    auton.withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();

    robotContainer.swerve.setBrakeMode();
    robotContainer.elevator.setBrakeMode();
    robotContainer.climber.setBrakeMode();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    ampTimer.reset();
    ampTimer.start();
    robotContainer.intake.run(0);

    robotContainer.swerve.setBrakeMode();
    robotContainer.elevator.setBrakeMode();
    robotContainer.climber.setBrakeMode();
  }

  @Override
  public void teleopPeriodic() {

    // Intake Rumble
    // if (robotContainer.controls.getDPadUpDriver()) { // intake.getStatorCurrent()
    // > 30.0
    // robotContainer.controls.setDriverRumble(0.8);
    // } else {
    // robotContainer.controls.setDriverRumble(0.0);
    // }

    if (robotContainer.intake.getStatorCurrent() > 30.0) {
      m_indaintake = true;
    } else {
      m_indaintake = false;
    }

    if (robotContainer.notehandler.getTorqueCurrent() > 11.0) {
      m_indahandler = true;
    } else {
      m_indahandler = false;
    }

    // Sensor Override
    if (robotContainer.controls.getPSPressedOperator()) {
      robotContainer.elevator.setSoftLimitsDisable();
      robotContainer.pivot.setSoftLimitsDisable();
      // m_manualPivot = true;
    }

    if (robotContainer.controls.getPSReleasedOperator()) {
      robotContainer.elevator.setSoftLimitsEnable();
      robotContainer.pivot.setSoftLimitsEnable();
    }

    if (robotContainer.controls.getSharePressedOperator()) {
      robotContainer.elevator.SetZero();
      robotContainer.pivot.setZero();
    }

    // SHOOTER + PRESETS

    // Shoot
    if (robotContainer.controls.getTriangleOperator()) {
      // Force Shoot
      m_pivotdown = true;
      m_pivotdownmemory = true;
      m_isshooting = true;

      if (robotContainer.shooter.getShooterRPM() < 10.0) {
        robotContainer.shooter.SetShooterRPM(1250);
      } else {
        robotContainer.shooter.setFeeder(1.0, true);
      }
    } else if ((Math.abs(robotContainer.controls.getOptionsandLeftY()) > 0.08)) {
      // Manual Control
      m_manualPivot = true;

      elevpivot = false;

      robotContainer.pivot.manual(robotContainer.controls.getOptionsandLeftY());
    } else if (m_manualPivot) {
      robotContainer.pivot.setAngle(robotContainer.pivot.getAngle());
      m_manualPivot = false;
    } else if (robotContainer.controls.getR1Driver()) {
      // Driver Automatic Shoot
      m_isshooting = true;
      m_pivotdown = true;
      m_pivotdownmemory = true;

      if (robotContainer.swerve.VisionStatus() == true) {
        if (robotContainer.pivot.getAngle() > 28.0) {
          m_elevbackdownpivot = true;
          elevpivot = true;
        } else {
          elevpivot = false;
        }
        var angleTolerance = kAngleTolerance;
        if (robotContainer.visiondistance() > 7.5) {
          double shooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 0.0);
          double shooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 0.0);
          if (robotContainer.controls.getCircleDriver()
              || robotContainer.controls.getCircleOperator()) {
            shooterRPM = SmartDashboard.getNumber(kFJShooterRPMString, 0.0);
          }
          robotContainer.shooter.SetShooterRPM(shooterRPM);
          robotContainer.pivot.setAngle(shooterAngle);
          angleTolerance = kAngleTolerance * 2;
        } else {
          robotContainer.shooter.SetShooterRPM(
              robotContainer.shooter.getInterpolateVals(robotContainer.visiondistance()));
          robotContainer.pivot.setAngle(
              robotContainer.pivot.getInterpolateVals(robotContainer.visiondistance()));
        }

        if (robotContainer.pivot.AtAngle(angleTolerance)
            && robotContainer.shooter.AtRPM(kRpmTolerance)
            && Math.abs(robotContainer.swerve.getFacingError().getDegrees()) < kHeadingTolerance
            && !robotContainer.shooter.NoteNotDetected()) { // TODO: Change this tolerance value
          robotContainer.shooter.setFeeder(1.0, true);
        } else {
          if (!m_isintaking) {
            robotContainer.shooter.setFeeder(0.3, false);
          }
        }
      } else {
        if (robotContainer.pivot.getAngle() > 28.0) {
          m_elevbackdownpivot = true;
          elevpivot = true;
        } else {
          elevpivot = false;
        }
        if (robotContainer.shooter.getShooterRPM() < 10.0) {
          robotContainer.shooter.SetShooterRPM(3000);
          robotContainer.pivot.setAngle(37.5);
        } else {
          if (robotContainer.pivot.AtAngle(kAngleTolerance)
              && robotContainer.shooter.AtRPM(kRpmTolerance)) { // TODO: Change this tolerance value
            robotContainer.shooter.setFeeder(1.0, true);
          } else {
            robotContainer.shooter.setFeeder(0.0, false);
          }
        }
      }
    } else if (robotContainer.controls.getTriangleDriver()) { // 25.5 2150
      // SmartDash (Loft) Shot
      m_manualPivot = false;
      m_isshooting = true;
      m_isintaking = false;

      elevpivot = false;

      m_pivotdown = true;
      m_pivotdownmemory = true;

      double shooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 0.0);
      double shooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 0.0);
      if (robotContainer.controls.getCircleDriver()
          || robotContainer.controls.getCircleOperator()) {
        shooterRPM = SmartDashboard.getNumber(kFJShooterRPMString, 0.0);
      }
      robotContainer.shooter.SetShooterRPM(shooterRPM);
      robotContainer.pivot.setAngle(shooterAngle);

      if (robotContainer.pivot.AtAngle(kAngleTolerance)
          && robotContainer.shooter.AtRPM(kRpmTolerance)) { // TODO: Change this tolerance value
        robotContainer.shooter.setFeeder(1.0, true);
      } else {
        robotContainer.shooter.setFeeder(0.0, false);
      }
    } else if (robotContainer.controls.getSquareDriver()) {
      m_manualPivot = false;
      m_isshooting = true;
      m_isintaking = false;

      elevpivot = false;

      m_pivotdown = true;
      m_pivotdownmemory = true;

      robotContainer.shooter.SetShooterRPM(2150.0);
      robotContainer.pivot.setAngle(25.5);

      if (robotContainer.pivot.AtAngle(kAngleTolerance)
          && robotContainer.shooter.AtRPM(kRpmTolerance)) { // TODO: Change this tolerance value
        robotContainer.shooter.setFeeder(1.0, true);
      } else {
        robotContainer.shooter.setFeeder(0.0, false);
      }
    } else if (robotContainer.controls.getL1ButtonTester()) {
      double shooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 0.0);
      double shooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 0.0);
      if (robotContainer.controls.getCircleDriver()
          || robotContainer.controls.getCircleOperator()) {
        shooterRPM = SmartDashboard.getNumber(kFJShooterRPMString, 0.0);
      }
      robotContainer.shooter.SetShooterRPM(shooterRPM);
      robotContainer.pivot.setAngle(shooterAngle);
    } else if (robotContainer.controls.getDPadUpOperator()
        || robotContainer.controls.getDPadUpDriver()) {
      // Feed Shot

      m_isintaking = false;
      m_manualPivot = false;

      elevpivot = false;

      double shooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 0.0);
      double shooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 0.0);
      if (robotContainer.controls.getCircleDriver()
          || robotContainer.controls.getCircleOperator()) {
        shooterRPM = SmartDashboard.getNumber(kFJShooterRPMString, 0.0);
      }
      robotContainer.shooter.SetShooterRPM(shooterRPM);
      robotContainer.pivot.setAngle(shooterAngle);
    } else if (robotContainer.controls.getDPadRightOperator()
        || robotContainer.controls.getDPadRightDriver()) {
      // Stop Shoot

      m_isintaking = false;
      m_manualPivot = false;

      elevpivot = false;

      robotContainer.shooter.SetShooterRPM(0);
      robotContainer.pivot.setAngle(1.5);
    } else if (robotContainer.controls.getDPadLeftOperator()
        || robotContainer.controls.getDPadLeftDriver()) {
      // Adam Line-Truss shot

      m_isintaking = false;
      m_manualPivot = false;

      elevpivot = false;

      double shooterRPM = SmartDashboard.getNumber(kSFShooterRPMString, 0.0);
      double shooterAngle = SmartDashboard.getNumber(kSFShooterAngleString, 0.0);
      robotContainer.shooter.SetShooterRPM(shooterRPM);
      robotContainer.pivot.setAngle(shooterAngle);
    } else if (robotContainer.controls.getDPadDownOperator()
        || robotContainer.controls.getDPadDownDriver()) {
      // New Subwoofer Shot

      m_isintaking = false;
      m_manualPivot = false;
      elevpivot = true;
      m_elevbackdownpivot = true;

      robotContainer.shooter.SetShooterRPM(3000);
      robotContainer.pivot.setAngle(37.5);
    } else if (robotContainer.controls.getRightPaddle()) {
      // Spin up Shooter Vision
      m_isintaking = false;

      m_pivotdown = true;
      m_pivotdownmemory = true;

      if (robotContainer.swerve.VisionStatus() == true) {
        if (robotContainer.pivot.getAngle() > 28.0) {
          m_elevbackdownpivot = true;
          elevpivot = true;
        } else {
          elevpivot = false;
        }
        if (robotContainer.visiondistance() > 13) {
          double shooterRPM = SmartDashboard.getNumber(kSFShooterRPMString, 0.0);
          double shooterAngle = SmartDashboard.getNumber(kSFShooterAngleString, 0.0);
          robotContainer.shooter.SetShooterRPM(shooterRPM);
          robotContainer.pivot.setAngle(shooterAngle);
        } else if (robotContainer.visiondistance() > 7.5) {
          double shooterRPM = SmartDashboard.getNumber(kFShooterRPMString, 0.0);
          double shooterAngle = SmartDashboard.getNumber(kFShooterAngleString, 0.0);
          robotContainer.shooter.SetShooterRPM(shooterRPM);
          robotContainer.pivot.setAngle(shooterAngle);
        } else {
          robotContainer.shooter.SetShooterRPM(
              robotContainer.shooter.getInterpolateVals(robotContainer.visiondistance()));
          robotContainer.pivot.setAngle(
              robotContainer.pivot.getInterpolateVals(robotContainer.visiondistance()));
        }
      } else {
        if (robotContainer.shooter.getShooterRPM() < 10.0) {
          robotContainer.shooter.SetShooterRPM(3000);
          robotContainer.pivot.setAngle(37.5);
        }
        if (robotContainer.pivot.getAngle() > 28.0) {
          m_elevbackdownpivot = true;
          elevpivot = true;
        } else {
          elevpivot = false;
        }
      }
    } else if (!m_pivotdown && m_pivotdownmemory) {
      robotContainer.pivot.setAngle(1.5);
      robotContainer.shooter.SetShooterRPM(0);
      m_pivotdownmemory = false;
      elevpivot = false;
    } else {
      m_manualPivot = false;
      m_pivotdown = false;
      m_isshooting = false;
    }

    // INTAKE SHIT
    if (robotContainer.controls.getR2Operator() > 0.05) {
      // Intake to Amp (Operator)
      m_isintaking = true;
      if (robotContainer.controls.getR2Operator() > 0.5) {
        robotContainer.intake.run(0.5);
      } else {
        robotContainer.intake.run(robotContainer.controls.getR2Operator());
      }
      robotContainer.pivot.setAngle(0.0);
      // robotContainer.director.setDirectortoAmp(1.0);
      robotContainer.director.setDirectorCurrent(12);
      robotContainer.notehandler.setHandlerAmpCurrent(31);
    } else if (robotContainer.controls.getL2Operator() > 0.05) {
      // Intake to Feeder (Operator)
      m_isintaking = true;
      if (robotContainer.controls.getL2Operator() > 0.5) {
        robotContainer.intake.run(0.5);
      } else {
        robotContainer.intake.run(robotContainer.controls.getL2Operator());
      }
      robotContainer.pivot.setAngle(0.0);
      robotContainer.director.setDirectortoShooter(1.0);
      robotContainer.shooter.setFeeder(0.3, false);

    } else if (robotContainer.controls.getL2Driver() > 0.05) {
      // Intake to Amp (Driver)
      m_isintaking = true;
      if (robotContainer.controls.getL2Driver() > 0.5) {
        robotContainer.intake.run(0.5);
      } else {
        robotContainer.intake.run(robotContainer.controls.getL2Driver());
      }
      robotContainer.pivot.setAngle(0.0);
      robotContainer.director.setDirectorCurrent(12);
      // robotContainer.director.setDirectortoAmp(1.0);
      robotContainer.notehandler.setHandlerAmpCurrent(31); // INC AMP CURRENT
    } else if (robotContainer.controls.getR2Driver() > 0.05) {
      // Intake to Feeder (Driver)
      m_isintaking = true;
      if (robotContainer.controls.getR2Driver() > 0.5) {
        robotContainer.intake.run(0.5);
      } else {
        robotContainer.intake.run(robotContainer.controls.getR2Driver());
      }
      robotContainer.pivot.setAngle(0.0);
      robotContainer.director.setDirectortoShooter(1.0);
      robotContainer.shooter.setFeeder(0.3, false);

    } else if (robotContainer.controls.getCrossOperator()
        || robotContainer.controls.getCrossDriver()) {
      // Eject Intake
      m_isintaking = false;
      robotContainer.intake.run(-1.0);
      robotContainer.director.setDirectortoShooter(-1.0);
    } else {
      robotContainer.intake.run(0.0);
      robotContainer.director.setDirectortoStop();
      m_isintaking = false;
    }

    // NOTEHANDLER FUNCTIONS (AMP/TRAP)
    if (robotContainer.controls.getSquareOperator()) {
      // Score Amp Operator
      m_driveramp = false;
      robotContainer.notehandler.setHandlerTrap(
          -1.0); // TODO: CHANGE THE TRAP FUNCTION NAME (IT GOES TO AMP

    } else if (robotContainer.controls.getTouchpadOperator()) {
      m_driveramp = false;
      robotContainer.notehandler.setHandlerTrap(0.6);
    } else if (robotContainer.controls.getL1Driver()) {
      // Score Amp Driver
      m_driveramp = true;
      m_elevbackdown = true;
      if (Math.abs(robotContainer.elevator.getCurrentPosition() - 12.66) < 0.45) {
        robotContainer.notehandler.setHandlerTrap(-1.0);
      }
    } else if (robotContainer.controls.getLeftPaddle()) {
      m_driveramp = true;
      m_elevbackdown = true;
    } else {
      if (!m_isintaking) {
        robotContainer.notehandler.setHandlerStop();
      }
      m_driveramp = false;
    }
    // Tells the feeder to stop based on intaking and shooting flags
    if (!m_isintaking && !m_isshooting) {
      robotContainer.shooter.setFeeder(0.0, false);
    }

    // Elevator Commands

    double manualcommand = robotContainer.controls.getOperatorRightY();

    if (robotContainer.controls.getR1Operator()) {
      robotContainer.elevator.PositionCommand(12.66);
      m_elevatorhold = true;
    } else if (robotContainer.controls.getL1Operator()) {
      robotContainer.elevator.PositionCommand(0.0);
      m_elevatorhold = true;
      // } else if (m_isintaking) {
      // robotContainer.elevator.PositionCommand(2.0);
      // m_stowelevator = true;
      // } else if (!m_isintaking && m_stowelevator) {
      // // if (robotContainer.elevator.getCurrentPosition() > 2.2) {
      // //
      // robotContainer.elevator.PositionCommand(robotContainer.elevator.getCurrentPosition());
      // // } else {
      // robotContainer.elevator.PositionCommand(2.0);
      // // }
      // if (Math.abs(robotContainer.elevator.getCurrentPosition() - 2.0) < 0.1) {
      // m_stowelevator = false;
      // }
    } else if (m_elevbackdown && !m_driveramp) {
      robotContainer.elevator.PositionCommand(0.0);
      m_elevbackdown = false;
    } else if (m_elevbackdownpivot && !elevpivot) {
      robotContainer.elevator.PositionCommand(0.0);
      m_elevbackdownpivot = false;
    } else if (elevpivot) {
      robotContainer.elevator.PositionCommand(2.0);
    } else if (m_driveramp) {
      robotContainer.elevator.PositionCommand(12.66); // OLD 14.9
    } else if (Math.abs(manualcommand) > 0.1) {
      robotContainer.elevator.ManualCommand(
          Math.signum(manualcommand) * (manualcommand * manualcommand));
      m_elevatorhold = true;
    } else if (m_elevatorhold) {
      m_elevatorhold = false;
      robotContainer.elevator.PositionCommand(
          robotContainer.elevator.getCurrentPosition()
              + (robotContainer.elevator.ElevatorVelocity() * 0.1));
    }

    // Climber
    double climberCommand = robotContainer.controls.getOperatorLeftY();
    if (Math.abs(climberCommand) > 0.1) {
      robotContainer.climber.ClimberMove(climberCommand);
    } else {
      robotContainer.climber.HoldPosition();
    }
  }

  @Override
  public void disabledInit() {
    coaster.reset();
    coaster.start();
    brakemodeoneshot = false;

    if (auton != null) {
      auton.cancel();
      auton = null;
    }
    robotContainer.shooter.SetShooterRPM(0);
  }

  @Override
  public void disabledPeriodic() {

    if (coaster.get() > 10.0 && !brakemodeoneshot) {
      robotContainer.climber.setCoastMode();
      robotContainer.elevator.setCoastMode();
      robotContainer.swerve.setCoastMode();
      // robotContainer.pivot.setCoastMode();
      brakemodeoneshot = true;
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
