package com.robojackets.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.robojackets.RobotContainer;
import com.robojackets.lib.choreo.AutoMirroredTrajectory;
import com.robojackets.lib.trajectory.TrajectoryManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AutonChooser {
  private static SendableChooser<CommandBasedAuton> m_autonChooser = new SendableChooser<>();
  private static SendableChooser<Supplier<ChoreoTrajectory>> m_pathChooser =
      new SendableChooser<>();

  public static void Init(RobotContainer io) {
    m_autonChooser.setDefaultOption("00 - None", new EmptyCommandAuton());
    m_autonChooser.addOption("01 - ShootnScoot2Piece", new ShootnScoot2Piece(io));
    // m_autonChooser.addOption("02 - South 4 Far", new South4FarAuton(io));
    m_autonChooser.addOption("02 - South 3 Red", new South4RedAuton(io));
    m_autonChooser.addOption("02 - South 3 Blue", new South4BlueAuton(io));
    m_autonChooser.addOption("04 - South 3 Blue Switcheroo", new South4BlueSwitcheraoo(io));
    m_autonChooser.addOption("04 - South 3 Red Switcheroo", new South4RedSwitcheraoo(io));
    m_autonChooser.addOption("05 - South 3 Blue Contested", new South4BlueContested(io));
    m_autonChooser.addOption("05 - South 3 Red Contested", new South4RedContested(io));
    m_autonChooser.addOption("06 - Center 4 Blue Contested", new Center4BlueContested(io));
    m_autonChooser.addOption("06 - Center 4 Red Contested", new Center4RedContested(io));
    m_autonChooser.addOption("07 - Center 6 Blue Contested", new Center6BlueContested(io));
    m_autonChooser.addOption("08 - BLUE Amp Center 2 Grab", new BlueAmpThingyAuton(io));
    m_autonChooser.addOption("08 - RED Amp Center 2 Grab", new RedAmpThingyAuton(io));

    // m_autonChooser.addOption("03 - Sequencer Test", new SequencerTest(io));

    m_autonChooser.addOption(
        "98 - Follow Path",
        new FollowTrajectoryCommandAuton(
            io, () -> new AutoMirroredTrajectory(m_pathChooser.getSelected().get())));

    var trajectoryArray = TrajectoryManager.getInstance().getTrajectories();
    trajectoryArray.sort((a, b) -> a.compareToIgnoreCase(b));

    for (var traj : trajectoryArray) {
      m_pathChooser.addOption(traj, () -> Choreo.getTrajectory(traj));
    }

    SmartDashboard.putData("Choose Auton", m_autonChooser);
    SmartDashboard.putData("Choose Path", m_pathChooser);
  }

  public static Command getSelectedAuton() {
    return m_autonChooser.getSelected().build();
  }
}
