package com.robojackets;

import com.robojackets.oi.ControlMode;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public final class Constants {

  private Constants() {}

  public static final double loopPeriodSecs = 0.02;

  public static final Mode currentMode = Mode.REAL;

  public static final ControlMode DEFAULT_CONTROL_MODE = ControlMode.SPEAKER_MODE;

  public static final double WINCH_DEADBAND = 0.1;

  public static Measure<Velocity<Distance>> DRIVE_MAX_LINEAR_VELOCITY = Units.FeetPerSecond.of(16);
  public static Measure<Velocity<Angle>> DRIVE_MAX_ANGULAR_VELOCITY =
      Units.RotationsPerSecond.of(1);

  public static Measure<Velocity<Distance>> maxLinearVelocity(
      ControlMode controlMode, boolean slow) {
    switch (controlMode) {
      case SOURCE_PASS_MODE:
        if (!slow) {
          return DRIVE_MAX_LINEAR_VELOCITY.times(0.5);
        }
        break;
      default:
        if (slow) {
          return DRIVE_MAX_LINEAR_VELOCITY.times(0.66);
        }
    }

    return DRIVE_MAX_LINEAR_VELOCITY;
  }

  public static Measure<Velocity<Angle>> maxAngularVelocity(ControlMode controlMode, boolean slow) {
    switch (controlMode) {
      case SOURCE_PASS_MODE:
        if (!slow) {
          return DRIVE_MAX_ANGULAR_VELOCITY.times(0.5);
        }
        break;
      default:
        if (slow) {
          return DRIVE_MAX_ANGULAR_VELOCITY.times(0.66);
        }
    }

    return DRIVE_MAX_ANGULAR_VELOCITY;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
