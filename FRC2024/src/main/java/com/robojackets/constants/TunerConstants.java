package com.robojackets.constants;

import com.robojackets.subsystems.swerve.CTRESwerveDrive;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Preferences;

public class TunerConstants {
  public CTRESwerveDrive DriveTrain = null;
  public Measure<Velocity<Distance>> MaxVelocity = null;

  private static TunerConstants instance;

  public static TunerConstants getInstance() {
    if (instance == null) {
      if (Preferences.getString("Robot", "Comp").equals("Comp")) {
        instance = new CompTunerConstants();
      } else {
        instance = new PracTunerConstants();
      }
    }
    return instance;
  }
}
