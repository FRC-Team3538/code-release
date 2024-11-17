package com.robojackets.constants;

import com.robojackets.ctre.SwerveDrivetrainConstants;
import com.robojackets.ctre.SwerveModuleConstants;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class TunerConstants {
  public SwerveDrivetrainConstants DrivetrainConstants = null;
  public SwerveModuleConstants[] ModuleConstants = null;
  public Measure<Velocity<Distance>> MaxVelocity = null;

  private static final TunerConstants instance = new CompTunerConstants();

  public static TunerConstants getInstance() {
    return instance;
  }
}
