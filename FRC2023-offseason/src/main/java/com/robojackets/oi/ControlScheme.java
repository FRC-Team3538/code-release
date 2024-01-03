package com.robojackets.oi;

import com.robojackets.lib.ArcadeDriveInput;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class ControlScheme {
  public ChassisSpeeds getTankDriverInput() {
    return new ChassisSpeeds();
  }

  public ChassisSpeeds getSwerveDriverInput() {
    return new ChassisSpeeds();
  }

  public ArcadeDriveInput getTankArcadeInput() {
    return new ArcadeDriveInput(0.0, 0.0);
  }

  public boolean SliderSetZero() {
    return false;
  }

  public boolean ElbowSetZero() {
    return false;
  }

  public boolean sensorOverrideEnable() {
    return false;
  }

  public boolean sensorOverrideDisable() {
    return false;
  }

  public boolean useRobotCentric() {
    return false;
  }

  // public boolean SetRobotCentric() {
  //   return false;
  // }

  // public boolean SetFieldCentric() {
  //   return false;
  // }

  public boolean resetYaw() {
    return false;
  }

  public boolean toggleYawLock() {
    return false;
  }

  public double getIntakeCommand() {
    return 0.0;
  }

  public double getSliderCommand() {
    return 0.0;
  }

  public double getElbowCommand() {
    return 0.0;
  }

  public boolean DecElbowCommand() {
    return false;
  }

  public boolean IncElbowCommand() {
    return false;
  }

  public boolean getPresetShelf() {
    return false;
  }

  public boolean getPresetHighMuskegon() {
    return false;
  }

  public boolean getPresetHighMilford() {
    return false;
  }

  public boolean getPresetGround() {
    return false;
  }

  public boolean getPresetStow() {
    return false;
  }

  public boolean getPresetMid() {
    return false;
  }

  public boolean getPresetMidLaunch() {
    return false;
  }

  public boolean getPresetLowScore() {
    return false;
  }

  public String getName() {
    return "NONE";
  }
}
