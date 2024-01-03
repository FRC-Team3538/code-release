package com.robojackets;

import com.robojackets.lib.UnitConversion;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

public final class Constants {

  private Constants() {}

  private static final RobotType robot;
  public static final double loopPeriodSecs = 0.02;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  private static final DriveType drive;

  private static final String robotID;

  static {
    switch (Preferences.getString("ROBOT", "NONE")) {
      case "REAL":
        robot = RobotType.ROBOT_REAL;
        break;
      case "ROVER":
        robot = RobotType.ROBOT_ROVER;
        break;
      case "SIMBOT":
        robot = RobotType.ROBOT_SIMBOT;
        break;
      case "NONE":
      default:
        Preferences.setString("ROBOT", "NONE");
        robot = RobotType.ROBOT_NONE;
    }

    switch (Preferences.getString("DRIVE", "NONE")) {
      case "TANK":
        drive = DriveType.TANK;
        break;
      case "SWERVE":
        drive = DriveType.SWERVE;
        break;
      case "NONE":
      default:
        Preferences.setString("DRIVE", "NONE");
        drive = DriveType.NONE;
    }

    robotID = Preferences.getString("ID", "CHANGE ME");

    if (robotID.equals("CHANGE ME")) {
      Preferences.setString("ID", "CHANGE ME");
    }
  }

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_REAL;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_REAL:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static String getID() {
    return robotID;
  }

  public static DriveType getDrivetrain() {
    return drive;
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_REAL, "/media/sda1/", RobotType.ROBOT_ROVER, "/media/sda1/");

  public static enum RobotType {
    ROBOT_REAL,
    ROBOT_ROVER,
    ROBOT_SIMBOT,
    ROBOT_NONE
  }

  public static enum DriveType {
    TANK,
    SWERVE,
    NONE
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static final class TankDrive {
    private TankDrive() {}

    public static final double MaxLinearVelocityMetersPerSecond = 0;
    public static final double MaxAngularVelocityRadiansPerSecond = 0;
  }

  public static final class SwerveDrive {
    private SwerveDrive() {}

    public static final double MaxLinearVelocityMetersPerSecond =
        14 * UnitConversion.METERS_PER_FOOT;
    public static final double MaxAngularVelocityRadiansPerSecond =
        1 * UnitConversion.RADIANS_PER_ROTATION;
  }
}
