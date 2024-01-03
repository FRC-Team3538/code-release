package com.robojackets.auto;

import com.robojackets.RobotContainer;
import com.robojackets.lib.trajectory.TrajectoryManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class AutonChooser {
  private static SendableChooser<Function<RobotContainer, Auton>> m_autonChooser =
      new SendableChooser<>();

  public static void Init() {
    m_autonChooser.setDefaultOption("00 - None", io -> new EmptyAuton());
    // m_autonChooser.addOption("01 - Score And Bridge", io -> new ScoreAndBridge(io));
    // m_autonChooser.addOption("02 - BigBoi", io -> new BigBoi(io));
    // m_autonChooser.addOption("03 - BiggerBoiLeft", io -> new BiggerBoiLeft(io));
    // m_autonChooser.addOption("04 - BiggerBoiRight", io -> new BiggerBoiRight(io));
    // m_autonChooser.addOption("05 - BiggestBoi", io -> new BiggestBoi(io));
    // m_autonChooser.addOption("06 - BigBoiLefty", io -> new BigBoiLefty(io));
    // m_autonChooser.addOption("07 - BigBoiRighty", io -> new BigBoiRighty(io));
    // m_autonChooser.addOption("85 - Turn180", io -> new Turn180(io));

    m_autonChooser.addOption("50 - Clean - 3", io -> new CleanSide3gpAuton(io));
    // m_autonChooser.addOption("50 - Bump - 2.5", io -> new BumpSide3gpAuton(io));
    // m_autonChooser.addOption("50 - Bump - 2 - Balance", io -> new BumpSide2gpBalance(io));
    // m_autonChooser.addOption("50 - Clean - 2.5", io -> new CleanSide2gpAuton(io));
    // m_autonChooser.addOption("50 - Bump -3", io -> new FastBumpSide3gpAuton(io));

    // for (var traj : TrajectoryManager.getInstance().getPPTrajectories()) {
    //   m_autonChooser.addOption(
    //       String.format("99 - Follow Path: %s", traj), io -> new FollowPathAuton(io, traj));
    // }

    // var trajectoryArray = TrajectoryManager.getInstance().getTrajectories();
    // trajectoryArray.sort((a, b) -> a.compareToIgnoreCase(b));

    // for (var traj : trajectoryArray) {
    //   m_autonChooser.addOption(
    //       String.format("98 - Follow Path: %s", traj.substring(0, traj.length() - 5)),
    //       io ->
    //           new FollowCustomTrajectoryAuton(
    //               io, List.of(TrajectoryManager.getInstance().getTrajectory(traj))));
    // }

    m_autonChooser.addOption(
        "Close Side Paths",
        io ->
            new FollowCustomTrajectoryAuton(
                io,
                List.of(
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Red Grid 1 to Piece 1.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Red Piece 1 to Grid 3.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Red Grid 3 to Piece 2.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Red Piece 2 to Grid 2.path.json")),
                List.of(
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Blue Grid 1 to Piece 1.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Blue Piece 1 to Grid 3.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Blue Grid 3 to Piece 2.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("clear-project-Blue Piece 2 to Grid 2.path.json"))));

    m_autonChooser.addOption(
        "Bump Side Paths",
        io ->
            new FollowCustomTrajectoryAuton(
                io,
                List.of(
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Red Grid 9 to Piece 4.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Red Piece 4 to Grid 8.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Red Grid 8 to Piece 3.path.json")),
                List.of(
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Blue Grid 9 to Piece 4.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Blue Piece 4 to Grid 8.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Blue Grid 8 to Piece 3.path.json"),
                    TrajectoryManager.getInstance()
                        .getTrajectory("fast-bump-project-Blue Piece 3 to Grid 8.path.json"))));

    SmartDashboard.putData("Choose Auton", m_autonChooser);
  }

  private static Function<RobotContainer, Auton> selectedAutonFunc;
  private static Auton selectedAuton = new EmptyAuton();

  public static Auton getSelectedAuton(RobotContainer io) {
    SmartDashboard.putBoolean("Auton/Loaded", false);
    var thisAutonFunc = m_autonChooser.getSelected();
    if (thisAutonFunc != selectedAutonFunc) {
      selectedAutonFunc = thisAutonFunc;
      if (selectedAutonFunc != null) {
        try {
          selectedAuton = selectedAutonFunc.apply(io);
        } catch (Exception ex) {
          DriverStation.reportError(ex.getLocalizedMessage(), ex.getStackTrace());
          selectedAuton = new EmptyAuton();
        }
      } else {
        selectedAuton = new EmptyAuton();
      }
      Logger.recordOutput("Auton/name", selectedAuton.getClass().getSimpleName());
    }

    SmartDashboard.putBoolean("Auton/Loaded", true);
    return selectedAuton;
  }

  public static void clearSelectedAuton() {
    SmartDashboard.putBoolean("Auton/Loaded", false);
    selectedAuton = new EmptyAuton();
  }
}
