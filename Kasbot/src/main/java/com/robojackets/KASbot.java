// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets;

import static com.robojackets.oi.HeadingTarget.SPEAKER;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

import com.robojackets.auto.AutonChooser;
import com.robojackets.constants.FieldConstants;
import com.robojackets.constants.TunerConstants;
import com.robojackets.constants.VisionConstants;
import com.robojackets.lib.MacAddressUtil;
import com.robojackets.lib.RJLog;
import com.robojackets.oi.ControlModeManager;
import com.robojackets.oi.HeadingTarget;
import com.robojackets.subsystems.Drive;
import com.robojackets.subsystems.Intake;
import com.robojackets.subsystems.Pivot;
import com.robojackets.subsystems.RJTrigger;
import com.robojackets.subsystems.Shooter;
import com.robojackets.subsystems.Vision;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class KASbot extends TimedRobot {

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandPS4Controller operator = new CommandPS4Controller(1);
  private final ControlModeManager controlModeManager = new ControlModeManager();

  private final Drive drive =
      new Drive(
          TunerConstants.getInstance().DrivetrainConstants,
          TunerConstants.getInstance().ModuleConstants);
  private final Vision vision = new Vision(drive, VisionConstants.camera);
  private final Intake intake = new Intake();
  private final Pivot pivot = new Pivot();
  private final Shooter shooter = new Shooter();
  private final RJTrigger trigger = new RJTrigger();

  private final RobotContainer robot =
      new RobotContainer(
          drive, vision, intake, pivot, shooter, trigger, driver, operator, controlModeManager);

  private AutonChooser autonChooser = new AutonChooser(robot);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public KASbot() {
    RJLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(false)
            .withCaptureNt(false)
            .withLogExtras(false)
            .withNtPublish(true)
            .withLogEntryQueueCapacity(1000));
    // RJLog.setPdh(new PowerDistribution());
    RJLog.setEnabled(true);

    PortForwarder.add(5800, "10.35.38.10", 5800);
    PortForwarder.add(1181, "10.35.38.10", 1181);
    PortForwarder.add(1182, "10.35.38.10", 1182);
    PortForwarder.add(1183, "10.35.38.10", 1183);
    PortForwarder.add(1184, "10.35.38.10", 1184);
    PortForwarder.add(1185, "10.35.38.10", 1185);
    PortForwarder.add(1186, "10.35.38.10", 1186);
    RobotController.setBrownoutVoltage(6.0);
    Threads.setCurrentThreadPriority(true, 1);

    // Record metadata
    RJLog.log("Metadata/GitRevision", String.valueOf(BuildConstants.GIT_REVISION));
    RJLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    RJLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    RJLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);
    RJLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    RJLog.log("Metadata/BuildUnixTime", String.valueOf(BuildConstants.BUILD_UNIX_TIME));
    switch (BuildConstants.DIRTY) {
      case 0:
        RJLog.log("Metadata/GitDirty", "Clean");
        break;
      case 1:
        RJLog.log("Metadata/GitDirty", "Dirty");
        break;
      default:
        RJLog.log("Metadata/GitDirty", "Error");
        break;
    }
    RJLog.log("Metadata/Robot MAC Address", MacAddressUtil.getMACAddress());

    // Log active commands
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommand(command, true, Optional.empty());
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommand(command, false, Optional.empty());
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command, Optional<Command> interruptor) -> {
              if (interruptor.isPresent()) logCommand(command, false, interruptor);
            });

    SmartDashboard.putData(field);

    autonomous().whileTrue(autonChooser.getSelectedAuton());
  }

  Map<String, Integer> commandCounts = new HashMap<>();

  private void logCommand(Command command, Boolean active, Optional<Command> maybeInterrupt) {
    String name = command.getName();
    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
    commandCounts.put(name, count);
    RJLog.log("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
    RJLog.log("CommandsAll/" + name, count > 0);

    if (maybeInterrupt.isPresent()) {
      Command interruptor = maybeInterrupt.get();
      String interruptorName = interruptor.getName();
      RJLog.log("CommandsAll/" + name + "/interruptor", interruptorName);
    }
  }

  @Override
  public void robotPeriodic() {
    robot.periodic();
    CommandScheduler.getInstance().run();

    RJLog.log("Robot/Mode", controlModeManager.getCurrentMode());

    var current_position = drive.getPose().getTranslation().toTranslation2d();
    var distance_to_pass_target =
        current_position.getDistance(FieldConstants.getTarget(robot.getAlliance(), SPEAKER));
    SmartDashboard.putNumber("Robot/DistanceToSpeaker", filter.calculate(distance_to_pass_target));

    field.setRobotPose(drive.getPose().toPose2d());
    field
        .getObject("target")
        .setPose(
            new Pose2d(
                FieldConstants.getTarget(null, HeadingTarget.SPEAKER), Rotation2d.fromDegrees(0)));

    RJLog.log("Alliance", robot.getAlliance());
  }

  private MedianFilter filter = new MedianFilter(50);
  private Field2d field = new Field2d();

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
