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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
 * import com.ctre.phoenixpro.configs.Slot0Configs; The VM is configured to automatically run this
 * class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project,
 * you must also update the build.gradle file in the project.
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

  boolean yawLockEnabled = true;

  int m_state = 0;
  private Auton auton;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    var logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("Robot", Constants.getRobot().toString());
    // logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    // logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          logger.addDataReceiver(new WPILOGWriter(folder));
        } else {
          logNoFileAlert.set(true);
        }
        logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        File wpilog_dir = new File(Filesystem.getOperatingDirectory(), "wpilogs");
        logger.addDataReceiver(new WPILOGWriter(wpilog_dir.getAbsolutePath()));
        logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    setUseTiming(Constants.getMode() != Mode.REPLAY);
    logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.getInstance()
              .recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
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
  }

  @Override
  public void robotPeriodic() {
    SignalManager.getInstance().refresh();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    Threads.setCurrentThreadPriority(true, 99);

    switch (Constants.getDrivetrain()) {
      case SWERVE:
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
        break;
      case TANK:
        // robotContainer.tank.periodic();
        break;
      case NONE:
      default:
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

    if (robotContainer.controls.toggleYawLock()) {
      yawLockEnabled = !yawLockEnabled;
    }
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

    Threads.setCurrentThreadPriority(true, 10);

    SmartDashboard.putBoolean("Field Oriented?", fieldOriented);

    SmartDashboard.putNumber("slider pos", robotContainer.arm.GetSliderPosition());

    SmartDashboard.putNumber("elbow pos", robotContainer.arm.getElbowPosition());

    SmartDashboard.putBoolean("slider LS", robotContainer.arm.sliderLimitSwitch.get());

    SmartDashboard.putNumber("preset state", m_state);

    SmartDashboard.putBoolean("YawLockEnabled?", yawLockEnabled);

    double imuPitch = robotContainer.swerve.ioInputs.getPitchRadians();
    double imuYaw = robotContainer.swerve.ioInputs.getYawRadians();

    SmartDashboard.putNumber("PitchPigeon", imuPitch * 180 / Math.PI);
    SmartDashboard.putNumber("YawPigeon", imuYaw * 180 / Math.PI);

    SmartDashboard.putNumber(
        "elbow enc", robotContainer.arm.elbowCoder.getAbsolutePosition().getValue() * 360.0);

    var absEncoder = robotContainer.arm.elbowCoder.getAbsolutePosition();
    SmartDashboard.putBoolean("Elbow Enc OK", absEncoder.getError().isOK());

    // var absEncoder = robotContainer.arm.elbowCoder.getAbsolutePosition();
    // SmartDashboard.putBoolean("Elbow Enc OK", absEncoder.getError().isOK());
  }

  @Override
  public void autonomousInit() {
    // NOTE: If working on elements field, set DS to Blue Alliance

    if (Math.abs(82.0 - robotContainer.arm.getElbowPosition()) > 10.0) {
      robotContainer.arm.syncElbowWithEncoder();
    }

    robotContainer.swerve.resetPose(
        AllianceFlipUtil.apply(new Pose2d(0.0, 0.0, Rotation2d.fromRotations(0.5))));

    auton = AutonChooser.getSelectedAuton(robotContainer);
    auton.Init();
    SmartDashboard.putData("Auton", auton);
    switch (Constants.getDrivetrain()) {
      case SWERVE:
        // TODO
        break;
      case TANK:
        // robotContainer.tank.setBrakeMode(true);
        break;
      case NONE:
      default:
    }
  }

  @Override
  public void autonomousPeriodic() {
    auton.Periodic();
  }

  private double targetYaw = 0.0;
  private Timer yawTimer = new Timer();

  @Override
  public void teleopInit() {
    auton = null;
    AutonChooser.clearSelectedAuton();

    targetYaw = robotContainer.swerve.ioInputs.getYawRadians();
    yawTimer.reset();
    yawTimer.start();

    switch (Constants.getDrivetrain()) {
      case SWERVE:
        // TODO
        break;
      case TANK:
        // robotContainer.tank.setBrakeMode(true);
        break;
      case NONE:
      default:
    }
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

    Logger.getInstance().recordOutput("Control Scheme", robotContainer.controls.getName());

    switch (Constants.getDrivetrain()) {
      case SWERVE:
        var input = robotContainer.controls.getSwerveDriverInput();
        Logger.getInstance()
            .recordOutput(
                "Driver Input",
                new double[] {
                  input.vxMetersPerSecond, input.vyMetersPerSecond, input.omegaRadiansPerSecond
                });

        // yaw lock
        if (input.omegaRadiansPerSecond == 0.0 && yawLockEnabled) {
          if (yawTimer.get() < 0.3) {
            targetYaw = robotContainer.swerve.ioInputs.getYawRadians();
          } else {
            double yaw = robotContainer.swerve.ioInputs.getYawRadians();
            double yawError = targetYaw - yaw;
            double yawKP = 1.0;
            double yawLockCmd = yawError * yawKP;
            double yawLockCmdMax = 0.2 * Constants.SwerveDrive.MaxAngularVelocityRadiansPerSecond;

            // Limit the yawLockCmd
            if (yawLockCmd > yawLockCmdMax) {
              yawLockCmd = yawLockCmdMax;
            } else if (yawLockCmd < -yawLockCmdMax) {
              yawLockCmd = -yawLockCmdMax;
            }

            input.omegaRadiansPerSecond += yawLockCmd;
          }
        } else {
          yawTimer.reset();
          yawTimer.start();
        }
        double pitchdeg = 0;
        double rolldeg = 0;
        double forwardcorrect = 0;
        double sidewayscorrect = 0;
        boolean tipped = false;

        if (fieldOriented) {
          robotContainer.swerve.DriveFieldCentric(AllianceFlipUtil.apply(input));
        } else {

          pitchdeg = (robotContainer.swerve.ioInputs.getPitchRadians() * 180.0 / Math.PI);
          rolldeg = (robotContainer.swerve.ioInputs.getRollRadians() * 180.0 / Math.PI);

          tipped = false;

          if (Math.abs(pitchdeg) > 7.0 && Math.abs(pitchdeg) < 45.0) {
            forwardcorrect = 0.75 * pitchdeg / Math.abs(pitchdeg);
            tipped = true;
          }
          if (Math.abs(rolldeg) > 7.0 && Math.abs(rolldeg) < 45.0) {
            sidewayscorrect = -0.75 * rolldeg / Math.abs(rolldeg);
            tipped = true;
          }

          if (tipped == true) {
            robotContainer.swerve.DriveRobotCentric(
                new ChassisSpeeds(forwardcorrect, sidewayscorrect, 0.0));
          } else {
            robotContainer.swerve.DriveRobotCentric(input);
          }
        }

        break;
      case TANK:
        // robotContainer.tank.Arcade(robotContainer.controls.getTankArcadeInput());
        break;
      case NONE:
      default:
    }
  }

  @Override
  public void disabledInit() {
    Braketimer.reset();
    Braketimer.start();
    BrakeModeOneShot = false;
    switch (Constants.getDrivetrain()) {
      case SWERVE:
        robotContainer.swerve.DriveFieldCentric(new ChassisSpeeds(0, 0, 0));
        break;
      case TANK:
        // robotContainer.tank.Arcade(new ArcadeDriveInput(0, 0));
        break;
      case NONE:
      default:
    }
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
