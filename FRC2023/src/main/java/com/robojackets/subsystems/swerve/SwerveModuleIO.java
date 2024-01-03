package com.robojackets.subsystems.swerve;

import com.robojackets.subsystems.gearbox.GearboxIO.GearboxIOInputs;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public class SwerveModuleIO {
  public void setDesiredState(SwerveModuleState desiredState) {}

  public void Stop() {}

  public void updateInputs(
      SwerveModuleIOInputs ioInputs, GearboxIOInputs driveInputs, GearboxIOInputs azimuthInputs) {}

  @AutoLog
  public static class SwerveModuleIOInputs {
    double angleRadians;
    double distanceMeters;
    double velocityMetersPerSecond;
    double externalAngleRadians;
  }

  public void SimPeriodic() {}
}
