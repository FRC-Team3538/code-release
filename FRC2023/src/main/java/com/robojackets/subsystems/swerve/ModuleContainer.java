package com.robojackets.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Consumer;

public class ModuleContainer {
  private final SwerveModule[] modules;

  public ModuleContainer(SwerveModule... modules) {
    this.modules = modules;
  }

  public Translation2d[] getLocations() {
    return new Translation2d[] {
      modules[0].getLocation(),
      modules[1].getLocation(),
      modules[2].getLocation(),
      modules[3].getLocation()
    };
  }

  public SwerveModulePosition[] getOdometryData() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()
    };
  }

  public void Drive(SwerveModuleState[] states) {
    modules[0].Drive(states[0]); // Drivecommands coming from SwerveModule
    modules[1].Drive(states[1]);
    modules[2].Drive(states[2]);
    modules[3].Drive(states[3]);
  }

  public void forEach(Consumer<SwerveModule> func) {
    for (var module : modules) {
      func.accept(module);
    }
  }
}
