package com.robojackets.auto;

import static com.robojackets.oi.HeadingTarget.SPEAKER;
import static com.robojackets.oi.StructureState.AIM_AT_SPEAKER;
import static com.robojackets.oi.StructureState.HOME;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.robojackets.RobotContainer;
import com.robojackets.constants.FieldConstants;
import com.robojackets.lib.trajectory.TrajectoryManager;
import com.robojackets.subsystems.Drive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class AutonChooser {
  private final SendableChooser<String> m_autonChooser = new SendableChooser<>();
  private final SendableChooser<String> m_pathChooser = new SendableChooser<>();

  private final Map<String, Command> commandMap;

  public AutonChooser(RobotContainer robot) {

    commandMap =
        Map.of(
            "00 - None", new EmptyCommandAuton(),
            "01 - Shoot? move?",
                simpleAuton(robot, () -> Choreo.getTrajectory(m_pathChooser.getSelected())),
            "98 - Follow Path",
                followTrajectory(
                    robot.drive, () -> Choreo.getTrajectory(m_pathChooser.getSelected())));

    m_autonChooser.setDefaultOption("00 - None", "00 - None");
    m_autonChooser.addOption("01 - Shoot? move?", "01 - Shoot? move?");

    m_autonChooser.addOption("98 - Follow Path", "98 - Follow Path");

    TrajectoryManager.getInstance().LoadTrajectories();

    var trajectoryArray = TrajectoryManager.getInstance().getTrajectories();
    trajectoryArray.sort((a, b) -> a.compareToIgnoreCase(b));

    for (var traj : trajectoryArray) {
      m_pathChooser.addOption(traj, traj);
    }

    m_pathChooser.setDefaultOption("No Path", "No Path");

    SmartDashboard.putData("Choose Auton", m_autonChooser);
    SmartDashboard.putData("Choose Path", m_pathChooser);
  }

  public Command getSelectedAuton() {
    return select(commandMap, () -> m_autonChooser.getSelected());
  }

  private Command followTrajectory(Drive drive, Supplier<ChoreoTrajectory> trajectorySupplier) {

    return sequence(
            print("Beginning Trajectory"), followTrajectoryInternal(drive, trajectorySupplier))
        .withName("Auton::FollowTrajectory");
  }

  private Command simpleAuton(RobotContainer robot, Supplier<ChoreoTrajectory> trajectorySupplier) {
    return sequence(
            print("Beginning Simple Auton"),
            deadline(
                waitSeconds(1.0).andThen(robot.fireCommand(false)),
                robot.drive.fieldCentricFacingTargetCommand(
                    () -> new ChassisSpeeds(),
                    () -> FieldConstants.getTarget(robot.getAlliance(), SPEAKER)),
                robot.aimCommand(AIM_AT_SPEAKER)),
            parallel(
                followTrajectoryInternal(robot.drive, trajectorySupplier), robot.aimCommand(HOME)))
        .withName("Auton::SimpleAuton");
  }

  private Command followTrajectoryInternal(
      Drive drive, Supplier<ChoreoTrajectory> trajectorySupplier) {
    return defer(
        () -> {
          var trajectory = trajectorySupplier.get();

          if (trajectory == null) {
            return drive.LockWheels();
          }

          return runOnce(() -> drive.resetPose(trajectory.getInitialPose()))
              .andThen(drive.FollowPath(trajectory))
              .andThen(drive.HoldPose(trajectory.getFinalPose()));
        },
        Set.of(drive));
  }
}
