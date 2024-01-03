// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets;

import com.robojackets.Constants.Mode;
import com.robojackets.auto.Auton;
import com.robojackets.auto.AutonChooser;
import com.robojackets.lib.SignalManager;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.subsystems.ArmSetpoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  public Timer Braketimer = new Timer();
  boolean BrakeModeOneShot = false;

  boolean IntakeForward = true;

  boolean fieldOriented = true;

  int m_state = 0;
  private Auton auton;

  private Rotation2d lastHeading = new Rotation2d();
  private boolean inHeadingLock = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    // Logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    // Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
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
        // String folder = Constants.logFolders.get(Constants.getRobot());
        // if (folder != null) {
        //   Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        // } else {
        //   logNoFileAlert.set(true);
        // }
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));

        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        File wpilog_dir = new File(Filesystem.getOperatingDirectory(), "wpilogs");
        Logger.addDataReceiver(new WPILOGWriter(wpilog_dir.getAbsolutePath()));
        Logger.addDataReceiver(new NT4Publisher());
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
    TrajectoryManager.getInstance().LoadPathPlannerTrajectories();
    AutonChooser.Init();
    SignalManager.getInstance().finalize();

    robotContainer.swerve.tareEverything();
  }

  @Override
  public void robotPeriodic() {
    SignalManager.getInstance().refresh();
    Threads.setCurrentThreadPriority(true, 1);

    robotContainer.swerve.periodic();

    if (robotContainer.controls.resetYaw()) {
      robotContainer.swerve.resetYaw(AllianceFlipUtil.apply(new Rotation3d()));
      // targetYaw = 0.0;
    }

    // NEW CODE UNDER WITH TOGGLE WHEN HES HOLDING
    if (robotContainer.controls.useRobotCentric()) {
      fieldOriented = false;
    } else {
      fieldOriented = true;
    }

    robotContainer.arm.periodic();

    if (robotContainer.controls.sensorOverrideEnable()) {
      robotContainer.arm.setSoftLimitsDisable();
      m_manualArm = true;
    }

    if (robotContainer.controls.sensorOverrideDisable()) {
      robotContainer.arm.setSoftLimitsEnable();
    }

    if (robotContainer.controls.SliderSetZero()) {
      robotContainer.arm.SliderSetZero();
    }
    if (robotContainer.controls.ElbowSetZero()) {
      robotContainer.arm.ElbowSetZero();
    }

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    Threads.setCurrentThreadPriority(true, 10);

    SmartDashboard.putBoolean("Field Oriented?", fieldOriented);

    SmartDashboard.putNumber("slider pos", robotContainer.arm.GetSliderPosition());

    SmartDashboard.putNumber("elbow pos", robotContainer.arm.getElbowPosition());

    SmartDashboard.putBoolean("slider LS", robotContainer.arm.sliderLimitSwitch.get());

    SmartDashboard.putNumber("preset state", m_state);

    try {
      Logger.recordOutput("Last Heading", lastHeading);
      Logger.recordOutput("Heading Lock", inHeadingLock);
    } catch (Exception ex) {
      DriverStation.reportError(ex.getLocalizedMessage(), ex.getStackTrace());
    }

    // double imuPitch = robotContainer.swerve.ioInputs.getPitchRadians();
    // double imuYaw = robotContainer.swerve.ioInputs.getYawRadians();

    // SmartDashboard.putNumber("PitchPigeon", imuPitch * 180 / Math.PI);
    // SmartDashboard.putNumber("YawPigeon", imuYaw * 180 / Math.PI);

    SmartDashboard.putNumber(
        "elbow enc", robotContainer.arm.elbowCoder.getAbsolutePosition().getValue() * 360.0);

    var absEncoder = robotContainer.arm.elbowCoder.getAbsolutePosition();
    SmartDashboard.putBoolean("Elbow Enc OK", absEncoder.getStatus().isOK());

    // var absEncoder = robotContainer.arm.elbowCoder.getAbsolutePosition();
    // SmartDashboard.putBoolean("Elbow Enc OK", absEncoder.getError().isOK());
  }

  @Override
  public void autonomousInit() {
    // NOTE: If working on elements field, set DS to Blue Alliance

    if (Math.abs(82.0 - robotContainer.arm.getElbowPosition()) > 10.0) {
      robotContainer.arm.syncElbowWithEncoder();
    }

    auton = AutonChooser.getSelectedAuton(robotContainer);
    auton.Init();
    SmartDashboard.putData("Auton", auton);
  }

  @Override
  public void autonomousPeriodic() {
    auton.Periodic();
  }

  // private double targetYaw = 0.0;
  // private Timer yawTimer = new Timer();

  @Override
  public void teleopInit() {
    auton = null;

    lastHeading = robotContainer.swerve.getPose().getRotation();
  }

  private boolean m_manualArm = false;
  private boolean m_goingToGround = false;
  private boolean m_goingTostow = false;

  @Override
  public void teleopPeriodic() {

    boolean bManualSilder = Math.abs(robotContainer.controls.getSliderCommand()) > 0.05;
    boolean bManualElbow = Math.abs(robotContainer.controls.getElbowCommand()) > 0.05;

    int mydebugfriend = 0;
    boolean hitdaswitcheroo = false;

    if (robotContainer.controls.getPresetGround()) {
      mydebugfriend = 1;
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.GROUND);
    } else if (robotContainer.controls.getPresetStow()) {
      mydebugfriend = 1;
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.STOW);
    } else if (robotContainer.controls.getPresetShelf()) {
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.SHELF);
      mydebugfriend = 1;
    } else if (robotContainer.controls.getPresetHighMilford()) {
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.HIGH_MILFORD);
      mydebugfriend = 1;
    } else if (robotContainer.controls.getPresetMid()) {
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.MID);
      mydebugfriend = 1;
    } else if (robotContainer.controls.getPresetHighMuskegon()) {
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.HIGH_MUSKEGON);
      mydebugfriend = 1;
    } else if (robotContainer.controls.getPresetLowScore()) {
      m_manualArm = false;
      robotContainer.arm.GoTo(ArmSetpoint.LOW_SCORE);
      mydebugfriend = 5;
    } else if (robotContainer.controls.DecElbowCommand()) {
      m_manualArm = false;
      robotContainer.arm.JogElbow(-7.0);
      mydebugfriend = 2;
      hitdaswitcheroo = true;
    } else if (robotContainer.controls.IncElbowCommand()) {
      m_manualArm = false;
      robotContainer.arm.JogElbow(7.0);
      mydebugfriend = 3;
      hitdaswitcheroo = false;
    } else if (bManualSilder || bManualElbow || m_manualArm) {
      m_manualArm = true;
      robotContainer.arm.SliderCmd(robotContainer.controls.getSliderCommand());
      robotContainer.arm.ElbowCmd(robotContainer.controls.getElbowCommand());
      mydebugfriend = 4;
    }

    SmartDashboard.putNumber("mydebugfriend", mydebugfriend);
    SmartDashboard.putBoolean("switcheraoo", hitdaswitcheroo);
    double intakeCommand = robotContainer.controls.getIntakeCommand();
    if (intakeCommand > 0.05) {
      IntakeForward = true;

    } else if (intakeCommand < -0.05) {
      IntakeForward = false;

    } else if (IntakeForward) {
      intakeCommand = 0.1;

    } else {
      intakeCommand = -0.1;
    }
    robotContainer.intakemotor.set(intakeCommand);

    Logger.recordOutput("Control Scheme", robotContainer.controls.getName());

    var input = robotContainer.controls.getSwerveDriverInput();

    if (!inHeadingLock) {
      lastHeading = robotContainer.swerve.getPose().getRotation();
    }

    if (fieldOriented) {
      if (input.omegaRadiansPerSecond != 0) {
        robotContainer.swerve.DriveFieldCentric(AllianceFlipUtil.apply(input));
      } else {
        robotContainer.swerve.DriveFieldCentricHoldingHeading(
            lastHeading, AllianceFlipUtil.apply(input));
      }
    } else {
      if (input.omegaRadiansPerSecond != 0) {
        robotContainer.swerve.DriveRobotCentric(input);
      } else {
        robotContainer.swerve.DriveRobotCentricHoldingHeading(lastHeading, input);
      }
    }

    inHeadingLock = input.omegaRadiansPerSecond == 0;
  }

  @Override
  public void disabledInit() {
    Braketimer.reset();
    Braketimer.start();
    BrakeModeOneShot = false;

    robotContainer.swerve.Cross();
  }

  @Override
  public void disabledPeriodic() {
    if (BrakeModeOneShot == false && Braketimer.get() > 3.0) {
      switch (Constants.getDrivetrain()) {
        case SWERVE:
          // TODO
          break;
        case TANK:
          // robotContainer.tank.setBrakeMode(false);
          break;
        case NONE:
        default:
      }
      BrakeModeOneShot = true;
    }

    AutonChooser.getSelectedAuton(robotContainer);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    robotContainer.swerve.simulationPeriodic();
  }
}
